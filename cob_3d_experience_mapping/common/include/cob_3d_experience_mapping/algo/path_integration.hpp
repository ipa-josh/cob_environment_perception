//path integration (input from odometry)

template<class TGraph, class TMapStates, class TMapTransformations, typename TState>
void insert_state(TGraph &graph, TMapStates &states, TMapTransformations &trans, TState &new_state) {
	new_state->set_node(graph.addNode());
	states.set(new_state->node(), new_state);
}

template<class TTransformation, class TGraph, class TMapTransformations, typename TState>
void insert_transistion(TGraph &graph, TMapTransformations &trans, const TState &new_state, TTransformation &link) {
	assert(link->src());
	
	if(new_state->id()<link->src()->id() && new_state->id()+10>link->src()->id()) {
		DBG_PRINTF("not inserted\n");
		return;
	}
	
	DBG_PRINTF("insert with dev %f\n", link->deviation().norm());
	trans.set(graph.addArc(new_state->node(), link->src()->node()), link);
}

template<class TGraph, typename TState>
void remove_state(TGraph &graph, TState &state) {
	if(!state) return;
	graph.erase(state->node()); //deletes also arcs
}

template<class TTransformation, class TGraph, class TContext, class TMapStates, class TMapTransformations>
void init(TGraph &graph, TContext &ctxt, TMapStates &states, TMapTransformations &trans)
{
	typedef typename TContext::TState TState;

	ctxt.virtual_state().reset(new TState(ctxt.id_generator().new_id()));
	ctxt.virtual_state()->dist_dev() = 1;
	ctxt.active_states().push_back(ctxt.virtual_state());
	insert_state(graph, states, trans, ctxt.virtual_state());
	
	ctxt.ft_new_slot();
	ctxt.virtual_transistion().reset(new TTransformation(ctxt.current_active_state()));
	ctxt.id_generator().register_modification(ctxt.virtual_state());

	//ctxt.virtual_state().reset(new TState);
	//insert_state<TTransformation>(graph, states, trans, ctxt.virtual_state(), ctxt.current_active_state());
}

template<class TEnergyFactor, class TGraph, class TContext, class TMapStates, class TMapTransformations, class TTransformation>
void path_integration(TGraph &graph, TContext &ctxt, TMapStates &states, TMapTransformations &trans, const TTransformation &odom, const TTransformation &odom_derv, const Eigen::Vector3f &dbg_pose)
{
	typedef typename TContext::TState TState;
	typedef typename TContext::TActList::iterator TIter;
	typedef typename TState::TArcOutIterator TArcIter_out;
	typedef typename TState::TArcInIterator TArcIter_in;
	typedef typename TState::TLink TLink;
	
	boost::lock_guard<boost::mutex> guard(ctxt.get_mutex());	//TODO: reduce locking area
	
	if(ctxt.needs_sort())
		std::sort(ctxt.active_states().begin(), ctxt.active_states().end(), sorting::energy_order<typename TContext::TActList::value_type>);
	assert( sorting::is_sorted(ctxt.active_states().begin(), ctxt.active_states().end(), sorting::energy_order<typename TContext::TActList::value_type>) );
		
	assert(ctxt.current_active_state()==*ctxt.active_states().begin());
	assert(ctxt.active_states().size()>0);
	assert(ctxt.current_active_state());
	
	debug::print_active_list(ctxt, dbg_pose);

	typename TState::TEnergy dev_increment = ctxt.add_odom(odom.get_data(), odom_derv.get_data());
	if(ctxt.virtual_state() && ctxt.virtual_transistion()) {		
		//ctxt.virtual_state()->set_dist_trv( ctxt.virtual_state()->dist_trv() - dev_increment/ctxt.normalize(odom.get_data()).norm() - 0.45f*(1-ctxt.ft_current_slot_similiarity()) );
		ctxt.virtual_state()->set_dist_trv( ctxt.virtual_state()->dist_trv() - dev_increment - 0.2f*(1-ctxt.ft_current_slot_similiarity()) );
		//dev_increment = ctxt.normalize(odom.get_data()).norm();
		ctxt.virtual_state()->dist_dev() +=   dev_increment;
		
		debug::print_changing_dist_V(ctxt, dev_increment);
	}

	if(!ctxt.virtual_state() || (ctxt.last_active_state()!=ctxt.current_active_state() && ctxt.current_active_state()->dist_trv()<=0) || (ctxt.current_active_state()!=ctxt.virtual_state() && ctxt.current_active_state()->dist_dev()<ctxt.last_dist_min() && ctxt.current_active_state()->dist_trv()<=0) || ctxt.virtual_state()->dist_trv()<=0 || ctxt.current_active_state()->dist_trv()<=-1) {
		debug::print_resetting_virtual_state(ctxt);
			
		bool exchange_active_state = false;
		if(ctxt.current_active_state()==ctxt.virtual_state()) {
			
			ctxt.virtual_transistion()->dst() = ctxt.current_active_state();
			ctxt.ft_add_features(ctxt.virtual_transistion());
		
			debug::print_insert_virtual_state(ctxt);
			
			exchange_active_state = true;
			
			ctxt.active_states().front()->is_active() = false;
			ctxt.active_states().front()->hops() = 0;
		}
		else {
			debug::print_relocalized(ctxt);
			
			if(ctxt.last_active_state()!=ctxt.current_active_state() && ctxt.last_active_state() && ctxt.current_active_state()) {
				
				bool exist=false;
				for(TArcIter_out ait(ctxt.current_active_state()->arc_out_begin(graph)); ait!=ctxt.current_active_state()->arc_out_end(graph); ++ait) {
					typename TIter::value_type opposite = states[ctxt.current_active_state()->opposite_node(graph, ait)];
					if(opposite==ctxt.virtual_transistion()->src()) {
						exist=true;
						
						assert(trans[ait]->dst());
						assert(trans[ait]->dst() == ctxt.current_active_state());
						ctxt.ft_add_features(trans[ait]);

						break;
					}
				}
				
				if(!exist) {
					ctxt.virtual_transistion()->dst() = ctxt.current_active_state();
					ctxt.ft_add_features(ctxt.virtual_transistion());
		
					insert_transistion(graph, trans, ctxt.current_active_state(), ctxt.virtual_transistion());
					ctxt.id_generator().register_modification(ctxt.current_active_state());
				}
				
				debug::print_relocalized(ctxt, exist, dbg_pose);
			}

			ctxt.remove_state(ctxt.virtual_state());
			remove_state(graph, ctxt.virtual_state());
		}
			
		typename TState::TEnergy old_dev = ctxt.virtual_state()->dist_dev();
		ctxt.virtual_state().reset(new TState(ctxt.id_generator().new_id()));
		ctxt.virtual_state()->reset_dist_trv();
		ctxt.virtual_state()->dist_dev() = std::min(old_dev, ctxt.current_active_state()->dist_dev()+ctxt.distance_sum()+dev_increment);//ctxt.current_active_state()->dist_dev() + offset;
		
		ctxt.on_new_virtual_state();
		
		ctxt.id_generator().register_modification(ctxt.virtual_state());
		
		ctxt.virtual_state()->dbg().pose_ = dbg_pose;
		
		insert_state(graph, states, trans, ctxt.virtual_state());
		ctxt.virtual_transistion().reset(new TTransformation(ctxt.current_active_state()));
		insert_transistion(graph, trans, ctxt.virtual_state(), ctxt.virtual_transistion());

		ctxt.last_active_state() = ctxt.current_active_state();
		
		if(exchange_active_state)
			ctxt.active_states().front() = ctxt.virtual_state();
		else
			ctxt.active_states().push_back(ctxt.virtual_state());
	}
	
	ctxt.ft_new_slot();
	
	ctxt.last_dist_min() = ctxt.current_active_state()->dist_dev();
	
	assert(ctxt.virtual_state());
	assert(ctxt.virtual_transistion());

	ctxt.virtual_transistion()->integrate(odom);
	if(ctxt.virtual_transistion()) DBG_PRINTF("virt trans B with dev %f\n", ctxt.virtual_transistion()->deviation().norm());
	
	{
		TIter begin = ctxt.active_states().begin();
		TIter end   = ctxt.active_states().end();
		
		//step 2: increase dist.
		for(TIter it=begin; it!=end; it++) {
			//if( (*it)->id() < ctxt.virtual_state()->id()-ctxt.param().min_age_ && (*it)->get_feature_prob()>0)
			//	continue;
			
			DBG_PRINTF("%d before dist (%f/%f) h:%d\n", (*it)->id(), (*it)->dist_trv(),(*it)->dist_dev(), (*it)->hops());	
	
			if( (*it)!=ctxt.virtual_state()) {
				typename TState::TEnergy dev_min = -1, sim_best = 0, E = 0, rel_best=0;
				
				TLink bef = (*it)->travel();
				const TLink &trv = (*it)->travel(odom);
				
				//for(TArcIter_in ait((*it)->arc_in_begin(graph)); ait!=(*it)->arc_in_end(graph); ++ait) {
				//		typename TIter::value_type opposite = states[(*it)->opposite_node(graph, ait)];
				for(TArcIter_out ait((*it)->arc_out_begin(graph)); ait!=(*it)->arc_out_end(graph); ++ait) {
					typename TState::TEnergy simb, devb, relb;
					typename TState::TEnergy sim, dev, rel;
					
					trans[ait]->directed(*it).transition_factor2(bef, ctxt.normalization_factor(), ctxt.deviation_sum(), ctxt.distance_relation(), simb, devb, relb);
					
					trans[ait]->directed(*it).transition_factor2(trv, ctxt.normalization_factor(), ctxt.deviation_sum(), ctxt.distance_relation(), sim, dev, rel);
					
					DBG_PRINTF("SIM %f   %f %f       %f %f\n",sim, dev, devb, rel, relb);
						
					dev = std::abs(dev-devb);
					rel = std::abs(rel-relb);
					
					if(simb<1 && sim>=1)
						dev = 0;
					
					if(dev_min<0 || dev<dev_min || (dev_min==dev && std::abs(1-sim)<std::abs(1-sim_best))) {
						dev_min = dev;
						sim_best= sim;
						rel_best= rel/ctxt.normalize(trans[ait]->get_data()).norm();
						E = rel_best/ctxt.normalize(trans[ait]->get_data()).norm();
						if(E!=E) E=0;
					}
				}
				(*it)->set_dist_trv(1-sim_best);
				
				if((*it)->dist_trv()<0) {
					//E = E / ctxt.distance_relation();
					for(TArcIter_in ait((*it)->arc_in_begin(graph)); ait!=(*it)->arc_in_end(graph); ++ait) {
						typename TIter::value_type opposite = states[(*it)->opposite_node(graph, ait)];
						if( !opposite->is_active() || opposite == ctxt.virtual_state() || opposite->dist_trv()<0 ) continue;
						
						DBG_PRINTF("dist_trv_var bef %f\n", opposite->dist_trv_var());
						opposite->dist_trv_var() = std::sqrt(1/(1/(opposite->dist_trv_var()*opposite->dist_trv_var()) + E*E));
						DBG_PRINTF("dist_trv_var aft %f\n", opposite->dist_trv_var());
					}
				}
				
				(*it)->dist_trv_var() += rel_best;
				(*it)->dist_trv_var() = std::max((*it)->dist_trv_var(), 0.1f);
				
				//const typename TState::TEnergy R = std::pow(1-1.f/(typename TState::TEnergy)active_states.size(), (typename TState::TEnergy)(*it)->hops()/(typename TState::TEnergy)active_states.size());
				//const typename TState::TEnergy R = std::pow(1-1.f/(typename TState::TEnergy)active_states.size(), std::sqrt((typename TState::TEnergy)(*it)->hops()));
				//const typename TState::TEnergy R = std::pow(1-1.f/(typename TState::TEnergy)active_states.size(), (typename TState::TEnergy)(*it)->hops()/(typename TState::TEnergy)active_states.size());
				//const typename TState::TEnergy R = std::pow(1-1.f/(typename TState::TEnergy)active_states.size(), (typename TState::TEnergy)(*it)->hops()/std::sqrt((typename TState::TEnergy)active_states.size()));
				typename TState::TEnergy R = std::pow(1-(1-(*it)->dist_trv_var())/(typename TState::TEnergy)active_states.size(), (typename TState::TEnergy)(*it)->hops()/std::sqrt((typename TState::TEnergy)active_states.size()));
				DBG_PRINTF("R=%f   var=%f states=%d hops=%d\n", R, (*it)->dist_trv_var(), (int)ctxt.active_states().size(), (*it)->hops());
				if((*it)->dist_trv()<-(*it)->dist_trv_var()) R=1;
				//(*it)->dist_dev() += std::max((typename TState::TEnergy)0, dev_min+dev_increment*(2*R-1) );
				(*it)->dist_dev() += std::max((typename TState::TEnergy)0, dev_min+dev_increment*R );
				
				DBG_PRINTF("%d changing dist(%f/%f) %f, %f, %f with R=%f\n\n", (*it)->id(),
					(*it)->dist_trv(),(*it)->dist_dev(), dev_min, dev_increment, sim_best, R);
			}
			
			assert( (*it)->dist_trv()==(*it)->dist_trv() );
			assert( (*it)->dist_dev()==(*it)->dist_dev() );
			assert( (*it)->dist_trv_var()==(*it)->dist_trv_var() );
		}
	}
	
	ctxt.new_ft_slot();
	
	const TIter begin = ctxt.active_states().begin();
	const TIter end   = ctxt.active_states().end();
	
	std::list<typename TIter::value_type> to_be_added;
	
	//step 0: update feature prob.
	for(TIter it=begin; it!=end; it++) {
		//if( (*it)->id() >= ctxt.virtual_state()->id()-ctxt.param().min_age_ )
		if( (*it) == ctxt.virtual_state())
			continue;
			
		typename TState::TEnergy ft_prob = (*it)->get_feature_prob();
			
		assert(ft_prob>=0 && ft_prob<=1);
		debug::print_ft_prob_before(**it, ft_prob);
		
		(*it)->dist_dev() -= ctxt.distance_relation() * std::min((typename TState::TEnergy)1, ft_prob);
		
		debug::print_ft_prob_after(**it, ft_prob);
	}
	
	//step 1: set min. dist.
	for(TIter it=begin; it!=end; it++) {
		typename TState::TEnergy border = 1;
		if((*it)->hops()<=4)
			border = 1-0.5f*(*it)->hops()/4.f;
		else
			border = 0.5f+0.4f*std::min(((*it)->hops()-4),4)/4.f;
		border = std::pow(0.9, active_states.size());
		if((*it)->dist_trv_var()>=border || (/*!(*it)->seen()&&*/(*it)->dist_trv()>(*it)->dist_trv_var()) )
		//if((*it)->dist_trv_var()>=(0.3f+0.7f*std::min(4, std::abs((*it)->hops()-4))/4.f) || (/*!(*it)->seen()&&*/(*it)->dist_trv()>(*it)->dist_trv_var()) )
		//if((*it)->dist_trv_var()>=(1-0.7f*std::min(4, (*it)->hops())/4.f) || (/*!(*it)->seen()&&*/(*it)->dist_trv()>(*it)->dist_trv_var()) )
			continue;
			
		typename TState::TEnergy dh_max = 0;
		int hops = -1;
		
		for(TArcIter_in ait((*it)->arc_in_begin(graph)); ait!=(*it)->arc_in_end(graph); ++ait) {
				typename TIter::value_type opposite = states[(*it)->opposite_node(graph, ait)];
				//if( (*it)->dist_trv()>ctxt.param().deviation_factor_/*trans[ait]->deviation()*/ || opposite->id() >= ctxt.virtual_state()->id()-ctxt.param().min_age_ /*|| (*it)->dist_h_out()<0.5-odom.deviation()-trans[ait]->deviation()*/ ) continue;
				//if( (*it)->dist_trv()>ctxt.param().deviation_factor_/*trans[ait]->deviation()*/ || opposite == ctxt.virtual_state() ) continue;
				if( /*ctxt.distance_relation()trans[ait]->deviation()*/ opposite == ctxt.virtual_state() ) continue;
				
				if( trans[ait]!=ctxt.virtual_transistion() && 
					(   (!opposite->is_active()&& (/*opposite->seen()||*/(*it)->dist_dev()<ctxt.virtual_state()->dist_dev()+3*ctxt.initial_distance())/*+ctxt.param().deviation_factor_*(*it)->hops()*/)
					 || ( opposite->is_active()&& opposite->dist_dev() > (*it)->dist_dev()/*&&(*it)->dist_trv_var()<opposite->dist_trv_var()*/) )
					//&& trans[ait]->directed(*it).transition_factor(odom, ctxt.param().prox_thr_)>odom.deviation()
					/*std::pow(trans[ait]->dist(ctxt.param().prox_thr_),2) + std::pow(opposite->dist_dev(),2)
					<
					(*it)->d2()*/
				) {	
					to_be_added.push_back(opposite);
					
					typename TState::TEnergy off = 0;
					//if(!opposite->is_active()  || (*it)->dist_trv_var()<opposite->dist_trv_var() ||  (*it)->hops()>opposite->hops())
					{
						//TODO: opposite->dist_trv() = std::min((typename TState::TEnergy)1, 1+(*it)->dist_trv());
						
						TArcIter_out best;
						typename TState::TEnergy dev_min = -1, sim_best = 0;
						for(TArcIter_out ait2((*it)->arc_out_begin(graph)); ait2!=(*it)->arc_out_end(graph); ++ait2) {
							typename TState::TEnergy sim, dev, rel;
							trans[ait2]->directed(*it).transition_factor2((*it)->travel(), ctxt.normalization_factor(), ctxt.deviation_sum(), ctxt.distance_relation(), sim, dev, rel);							
							if(dev_min<0 || dev<dev_min || (dev_min==dev && sim>sim_best)) {
								dev_min = dev;
								sim_best= sim;
								best = ait2;
							}
						}
				
						typename TLink::TLink er = (*it)->travel().get_data()+trans[best]->directed(*it).get_data();
						for(int i=0; i<er.rows(); i++) {
							if(er(i)>0) er(i) = std::max((typename TState::TEnergy)0, er(i)-trans[best]->deviation()(i));
							else 		er(i) = std::min((typename TState::TEnergy)0, er(i)+trans[best]->deviation()(i));
						}
						{
							typename TState::TEnergy sim, rel;
							trans[ait]->directed(opposite).transition_factor2(TLink( er ), ctxt.normalization_factor(), ctxt.deviation_sum(), ctxt.distance_relation(), sim, off, rel);
							if(sim<=0) {
								er.fill(0);
								off=0;
							}
							opposite->reset_dist_trv(TLink( er ), 1-sim);
							off = std::max(off, (typename TState::TEnergy)0.00001f);	//safety to prevent to become active state immediately
						}
						
						std::cout<<"dbg travel\n"<<(*it)->travel().get_data().transpose()<<"\n"<<trans[best]->directed(*it).get_data().transpose()<<"\n"<<opposite->travel().get_data().transpose()<<std::endl;
					}
					/*if((*it)->dist_trv_var()<opposite->dist_trv_var()) {
						opposite->dist_trv_var() = (*it)->dist_trv_var();
					}*/
					//opposite->dist_trv()  = std::min((typename TState::TEnergy)1, std::max((typename TState::TEnergy)-1, trans[ait]->dist(ctxt.param().prox_thr_)+(*it)->dist_trv()));
					
					if(opposite->dist_dev() >= (*it)->dist_dev()+off || !opposite->is_active()) {
						if(!opposite->is_active()) {
							opposite->dist_trv_var() = (*it)->dist_trv_var();
							opposite->hops() = 1+(*it)->hops();
						}
						else {
							opposite->hops() = std::max(opposite->hops(), 1+(*it)->hops());
							opposite->dist_trv_var() = std::min(opposite->dist_trv_var(), (*it)->dist_trv_var());
						}
						opposite->dist_dev() = (*it)->dist_dev()+off;
					}
					
					DBG_PRINTF("1 %d:%d setting dist %f/%f with %d hops (off %f)\n\n",
						opposite->id(), (*it)->id(),
						opposite->dist_trv(), opposite->dist_dev(),
						opposite->hops(), off
						);
				}
		}
		
	}
	
	for(TIter it=begin; it!=end; it++)
		(*it)->reset_feature();
	
	for(typename std::list<typename TIter::value_type>::iterator it=to_be_added.begin(); it!=to_be_added.end(); it++)
		ctxt.add_to_active(*it, true);
	
	//sort again
	std::sort(ctxt.active_states().begin(), ctxt.active_states().end(), sorting::energy_order<typename TContext::TActList::value_type>);
}

template<class TStateVector>
void reset_features(TStateVector &active_states)
{
	typedef typename TStateVector::iterator TIter;
	
	//reset features
	for(TIter it=active_states.begin(); it!=active_states.end(); it++) {
		(*it)->reset_feature();
	}
}

