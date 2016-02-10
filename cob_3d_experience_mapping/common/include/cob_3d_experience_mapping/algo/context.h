#pragma once

#include "../defs.h"
#include "../param.h"
#include <set>
#include <map>
#include <boost/thread.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/math/distributions/binomial.hpp>
#include "../helpers/network.h"

//! interfaces and implementations of cob_3d_experience_mapping
namespace cob_3d_experience_mapping {

	//! functions to sort active state list
	namespace sorting {
		
		template<class T>
		inline T mmax0(const T &v) {
			return std::max<T>(0, v);
		}
		
		//!< comparision of two states (by distance 1. deviation distance 2. travel distance)
		template<class TStatePtr>
		bool energy_order(const TStatePtr &a, const TStatePtr &b) {
			if(a->dist_dev() == b->dist_dev())
				return a->dist_trv() > b->dist_trv();
			return a->dist_dev() < b->dist_dev();
			//return a->dist_dev()+mmax0(a->dist_trv_var()-1) < b->dist_dev()+mmax0(b->dist_trv_var()-1);
		}
		
		//!< function to check if list is sorted (fallback for older C++ versions)
		template <class ForwardIterator, class Compare>
		bool is_sorted (ForwardIterator first, ForwardIterator last, Compare comp)
		{
			if (first==last) return true;
			ForwardIterator next = first;
			while (++next!=last) {
			if (comp(*next,*first))     // or, if (comp(*next,*first)) for version (2)
			  return false;
			++first;
			}
			return true;
		}
		
	}
	
	/*! \class Context
		\brief Representation of (active) map data

		Templated class which contains all active map data:
		 - active state list
		 - virtual state
		 - feature lookup to state
		 - some variables for algorithms
	*/
	template<class _TEnergy, class _TState, class _TFeature, class _TEnergyFactor, class _TTransform, class _TIdTsGenerator = SimpleIdTsGenerator<_TState, _TFeature> >
	class Context {
	public:
		typedef _TEnergy TEnergy;
		typedef _TState TState;
		typedef _TIdTsGenerator TIdTsGenerator;
		typedef _TFeature TFeature;
		typedef _TEnergyFactor TEnergyFactor;
		typedef _TTransform TTransform;
		typedef std::vector<typename TState::TPtr> TActList; 
		typedef typename TActList::iterator TActListIterator; 
		typedef Parameter<TEnergyFactor, typename _TTransform::TDist> TParameter;
		typedef std::map<typename TFeature::TID, typename TFeature::TPtr> FeatureMap;
		typedef boost::circular_buffer<typename TFeature::TID> FeatureBuffer;
		
		typedef std::vector<typename TFeature::TID> FeaturePerceivedSet;
		typedef std::deque<FeaturePerceivedSet> FeaturePerceivedHistory;
		
		static bool ft_perceived_in(const typename TFeature::TID &ft_id, const FeaturePerceivedSet &slot) {
			for(size_t i=0; i<slot.size(); i++)
				if(slot[i]==ft_id) return true;
			return false;
		}
		
		static TEnergy ft_slot_match(const FeaturePerceivedSet &slotA, const FeaturePerceivedSet &slotB) {
			TEnergy res = 0;
			for(size_t i=0; i<slotA.size(); i++)
				res += (ft_perceived_in(slotA[i], slotB)?1:0);
			return res/std::max((size_t)1, std::max(slotA.size(), slotB.size()));
		}
		
		TEnergy ft_current_slot_similiarity() const {
			if(ft_slots_.size()<2)
				return 1;
			if(ft_slots_[0].size()<1)
				return 1;
			TEnergy sim=0;
			//boost::math::binomial_distribution<TEnergy> distribution(ft_slots_.size()-1,0.5);
			for(size_t i=1; i<ft_slots_.size(); i++)
				sim += (float)i*2/((ft_slots_.size())*(ft_slots_.size()-1)) * /*boost::math::pdf(distribution, i-1)*/ ft_slot_match(ft_slots_[i], ft_slots_[0]);
			return sim;
		}
		
		TEnergy ft_chance_to_see(const typename TFeature::TID &ft_id) {
			int num=0;
			for(size_t i=1; i<ft_slots_.size(); i++)
				if(ft_perceived_in(ft_id, ft_slots_[i]))
					++num;
			return num/(TEnergy)(ft_slots_.size()-1);
		}
		
		void ft_add_features(typename TTransform::TPtr trans) {
			assert(current_active_state()==trans->dst());
			
			//boost::math::binomial_distribution<TEnergy> distribution(ft_slots_.size()-1,0.5);
			DBG_PRINTF("debug ft slots:\n");
			for(size_t i=0; i<ft_slots_.size(); i++) {
				DBG_PRINTF("ft slot %d (%f):  \t", (int)i, (float)(i-1)*2/((ft_slots_.size()-1)*ft_slots_.size()));//boost::math::pdf(distribution, i-1));
				for(size_t j=0; j<ft_slots_[i].size(); j++)
					DBG_PRINTF("%d\t", ft_slots_[i][j].id());
				DBG_PRINTF("\n");
			}
			
			DBG_PRINTF("size %d %d\n", (int)action_seq_.size(), (int)ft_slots_.size());
			assert(action_seq_.size() ==  ft_slots_.size());
			
			std::vector<TEnergy> sims;
			sims.push_back(0);
			for(size_t i=0; i<action_seq_.size()-1; i++) {
				TEnergy sim, dev;
				typename TTransform::TLink er;
				virtual_transistion()->transition_factor(TTransform(-action_seq_[i], typename TTransform::TLink(), virtual_state()), normalization_factor(), sim, dev, er);
				sims.push_back(sims.back()+sim);
				DBG_PRINTF("sims %f\n", sims.back());
			}
			
			std::map<typename TFeature::TID, bool> did_already;
			TEnergy prob_max = 0;
			bool registered = false;
			typename TFeature::TID id_max = -1;
			size_t i_max;
			for(size_t i=1; i<ft_slots_.size(); i++) {
				for(size_t j=0; j<ft_slots_[i].size(); j++) {
					visited_feature_class(ft_slots_[i][j].class_id());
					
					if(did_already.find(ft_slots_[i][j])!=did_already.end()) continue;
					
					const TEnergy prob = ft_chance_to_see(ft_slots_[i][j]);// * (0.5f + (float)(i-1)/((ft_slots_.size()-1)*ft_slots_.size()));
					did_already[ft_slots_[i][j]] = true;
				
					DBG_PRINTF("ft prob %f\n", prob);
					
					if(prob>=0.5) {
						size_t first = i;
						for(size_t k=i+1; k<ft_slots_.size(); k++)
							if(ft_perceived_in(ft_slots_[i][j], ft_slots_[k])) first = k;
						//features_[ft_slots_[i][j]]->visited(current_active_state().get(), trans, (sims[sims.size()-first-1]+sims[sims.size()-i]-sims[sims.size()-first-1])/2, sims[sims.size()-i]-sims[sims.size()-first-1]);
						features_[ft_slots_[i][j]]->visited(trans.get(), trans, sims[sims.size()-first-1], sims[sims.size()-i]-sims[sims.size()-first-1]);
						id_generator().register_modification(features_[ft_slots_[i][j]]);
						if(!registered) id_generator().register_modification(current_active_state());
						registered = true;
					}
					if(prob>prob_max) {
						prob_max = prob;
						id_max = ft_slots_[i][j];
						i_max = i;
					}
				}
			}
			
			DBG_PRINTF("prob_max %f\n", prob_max);
			
			if(prob_max>0 && prob_max<0.5) {
				size_t first = i_max;
				for(size_t k=i_max+1; k<ft_slots_.size(); k++)
					if(ft_perceived_in(id_max, ft_slots_[k])) first = k;
				features_[id_max]->visited(current_active_state().get(), trans, sims[sims.size()-first-1], sims[sims.size()-i_max]-sims[sims.size()-first-1]);
			
				id_generator().register_modification(features_[id_max]);
				if(!registered) id_generator().register_modification(current_active_state());
			}
		}
		
		void ft_new_slot() {
			//if(ft_slots_.size()<1 || ft_slots_.front().size()>0)
			//	ft_slots_.push_front(FeaturePerceivedSet());
				
			/*DBG_PRINTF("ft_new_slot:\n");
			for(size_t i=0; i<ft_slots_.size(); i++) {
				DBG_PRINTF("\tft slot %d:", (int)i);
				for(size_t j=0; j<ft_slots_[i].size(); j++) DBG_PRINTF("\t%d", ft_slots_[i][j]);
				DBG_PRINTF("\n");
			}*/
		}
		
		void visited_feature_class(const typename TState::TFeatureClass &ft_class) {
			if(current_active_state()) {
				current_active_state()->visited_feature_class(ft_class);
				id_generator().register_modification(current_active_state());
			}
		}
		
	private:
		TActList active_states_;		//!< active state list
		TParameter param_;			//!< parameter storage
		TIdTsGenerator id_generator_;
		TEnergy last_dist_min_;		//!< 
		typename TState::TPtr last_active_state_, virtual_state_;
		typename TTransform::TPtr virtual_transistion_;
		FeatureMap features_;
		FeatureBuffer last_features_;
		boost::mutex mtx_;
		bool needs_sort_;
		TEnergy distance_relation_sum_, distance_distance_sum_, distance_relation_num_;
		
		typename TTransform::TLink action_sum_, action_num_, deviation_sum_;
		std::vector<typename TTransform::TLink> action_seq_;
		FeaturePerceivedHistory ft_slots_;
		
	public:
		Context() : last_dist_min_(0), last_features_(10), needs_sort_(true) {
			action_num_.fill(0);
			action_sum_.fill(0);
			deviation_sum_.fill(0);
			distance_distance_sum_ = distance_relation_sum_ = distance_relation_num_ = 0;
			
			ft_slots_.push_front(FeaturePerceivedSet());
			
			/*action_num_(0) = 20;
			action_num_(2) = 20;
			action_sum_(0) = 20*0.5;
			action_sum_(2) = 20*0.4;
			
			distance_relation_ = 0.1f;*/
		}
		
		void new_ft_slot() {
			ft_slots_.push_front(FeaturePerceivedSet());
		}
		
		inline TEnergy initial_distance() const {
			return distance_relation();//distance_sum()*0.25f + distance_relation()*0.75f;
		}
		
		//TEnergy distance_relation() const {return distance_relation_;}
		TEnergy distance_relation() const {
			if(!distance_relation_num_) return 1;
			return distance_relation_sum_/distance_relation_num_;
		}
		
		TEnergy distance_sum() const {
			if(!distance_relation_num_) return 1;
			return distance_distance_sum_/distance_relation_num_;
		}
		
		typename TTransform::TLink deviation_sum() const {
			if(!distance_relation_num_) return deviation_sum_;
			return deviation_sum_/distance_relation_num_;
		}
		
		//!< check if sorting is needed (because feature was seen) and resets flag
		bool needs_sort() {
			bool tmp = needs_sort_;
			needs_sort_ = false;
			return tmp;
		}
		
		FeatureMap &get_features() {return features_;}
		TIdTsGenerator &id_generator() {return id_generator_;}
		
		void on_new_virtual_state() {
			{
				DBG_PRINTF("debug the seq.:\n");
				typename TTransform::TLink dev_v; dev_v.fill(0);
				TEnergy sim_sum = 0, dev_sum=0;
				for(size_t i=0; virtual_state() && i<action_seq_.size()-1; i++) {
					TEnergy sim, dev;
					typename TTransform::TLink er;
					virtual_transistion()->transition_factor(TTransform(-action_seq_[i], typename TTransform::TLink(), virtual_state()), normalization_factor(), sim, dev, er);
					sim_sum += sim;
					dev_sum += dev;
					dev_v   += er;
					DBG_PRINTF("  sim/dev: %f %f\n", sim, dev);
				}
				DBG_PRINTF("sim sum: %f %f\n", sim_sum, dev_sum);
				
				{
					TEnergy relation = 0, w;
					if(dev_sum>0) relation = dev_sum/sim_sum;
					//relation = sim_sum;
					if(relation!=relation || std::isinf(relation)) relation=0;
					w = 1;//relation/distance_relation();
					distance_distance_sum_ += w*sim_sum;
					distance_relation_sum_ += w*relation;
					deviation_sum_		   += w*dev_v;
					distance_relation_num_ += w;
				}
				DBG_PRINTF("distance_relation %f\n", distance_relation());
			}
			
			if(action_seq_.size()>1) {
				const typename TTransform::TLink tmp = action_seq_.back();
				action_seq_.clear();
				action_seq_.push_back(tmp);
			}
			if(ft_slots_.size()>1) {	//keep current set
				ft_slots_.erase(ft_slots_.begin()+1, ft_slots_.end());
			}
				
			/*DBG_PRINTF("on_new_virtual_state:\n");
			for(size_t i=0; i<ft_slots_.size(); i++) {
				DBG_PRINTF("\tft slot %d:", (int)i);
				for(size_t j=0; j<ft_slots_[i].size(); j++) DBG_PRINTF("\t%d", ft_slots_[i][j]);
				DBG_PRINTF("\n");
			}*/
		}
		
		TEnergy add_odom(const typename TTransform::TLink &odom, const typename TTransform::TLink &odom_derv) {
			typename TTransform::TLink tmp = odom_derv.cwiseAbs();
			//typename TTransform::TLink w = normalize(tmp);
			typename TTransform::TLink w;w.fill(1);
			action_sum_ += w.cwiseProduct(tmp);
			action_num_ += w;
			
			TEnergy dev_sum_bef=0, dev_sum_aft=0;
			typename TTransform::TLink vec_dev_sum_bef, vec_dev_sum_aft;
			vec_dev_sum_bef.fill(0);
			vec_dev_sum_aft.fill(0);
			
			for(size_t i=0; virtual_transistion() && i<action_seq_.size(); i++) {
				TEnergy sim, dev;
				typename TTransform::TLink er;
				virtual_transistion()->transition_factor(TTransform(-action_seq_[i], typename TTransform::TLink(), virtual_state()), normalization_factor(), sim, dev, er);
				vec_dev_sum_bef += er;
				dev_sum_bef += dev;
			}
			DBG_PRINTF("dev sum bef: %f\n", dev_sum_bef);
			action_seq_.push_back(odom);
			for(size_t i=0; virtual_transistion() && i<action_seq_.size(); i++) {
				TEnergy sim, dev;
				typename TTransform::TLink er;
				virtual_transistion()->transition_factor(TTransform(-action_seq_[i], typename TTransform::TLink(), virtual_state()), normalization_factor(), sim, dev, er);
				vec_dev_sum_aft += er;
				dev_sum_aft += dev;
			}
			DBG_PRINTF("dev sum aft: %f\n", dev_sum_aft);
			
			TEnergy running_dist = 0;
			for(size_t i=0; i<action_seq_.size(); i++) {
				DBG_PRINTF("seq:          %f %f %f\n", action_seq_[i](0), action_seq_[i](1), action_seq_[i](2));
				running_dist += normalize(action_seq_[i]).norm();
			}
			
			DBG_PRINTF("action_sum_:          %f %f %f\n", action_sum_(0), action_sum_(1), action_sum_(2));
			DBG_PRINTF("action_num_:          %f %f %f\n", action_num_(0), action_num_(1), action_num_(2));
			DBG_PRINTF("running_dist:         %f\n", running_dist);
			DBG_PRINTF("normalization_factor: %f %f %f %f\n", normalization_factor()(0), normalization_factor()(1), normalization_factor()(2), normalization_factor()(3));
			DBG_PRINTF("deviation_sum:        %f %f %f %f\n", deviation_sum()(0), deviation_sum()(1), deviation_sum()(2), deviation_sum()(3));
			if(ft_slots_.size()>0) DBG_PRINTF("ft_current_slot:      %f\n", ft_current_slot_similiarity());
			
			if(virtual_transistion()) {
				virtual_transistion()->deviation() = vec_dev_sum_bef;
			}

			return dev_sum_aft-dev_sum_bef;
			/*
			if(!virtual_transistion()) return 0;
			
			
			DBG_PRINTF("trans:          %f %f %f\n", virtual_transistion()->get_data()(0), virtual_transistion()->get_data()(1), virtual_transistion()->get_data()(2));
			DBG_PRINTF("go for: %f <-> %f\n", running_dist, normalize(virtual_transistion()->get_data()+odom).norm());
			
			TEnergy sim, dev;
			virtual_transistion()->transition_factor(TTransform(-odom, virtual_state()), normalization_factor(), sim, dev);
			
			return dev;
			
			return std::log(
					running_dist
					/
					normalize(virtual_transistion()->get_data()+odom).norm()
				) / std::log(odom.rows());*/
		}
		
		typename TTransform::TLink normalization_factor() const {
			typename TTransform::TLink tmp = action_sum_;
			for(int i=0; i<tmp.rows(); i++)
				if(tmp(i)==0 || action_num_(i)==0) tmp(i)=1;
				else tmp(i) /= action_num_(i);
			return tmp;
		}
		
		typename TTransform::TLink normalize2(const typename TTransform::TLink &link) const {
			typename TTransform::TLink tmp = action_num_.cwiseProduct(action_num_);
			for(int i=0; i<tmp.rows(); i++)
				if(action_sum_(i)==0) tmp(i)=1;
				else tmp(i) /= action_sum_(i)*action_sum_(i);
			return link.cwiseProduct(tmp);
		}
		
		typename TTransform::TLink normalize(const typename TTransform::TLink &link) const {
			typename TTransform::TLink tmp = action_num_;
			for(int i=0; i<tmp.rows(); i++)
				if(action_sum_(i)==0) tmp(i)=1;
				else tmp(i) /= action_sum_(i);
			return link.cwiseProduct(tmp);
		}
		
		//!< add a state to the active state list + init. variables + (init. distances if needed)
		void add_to_active(typename TState::TPtr &state, const bool already_set=false) {
			if(state->id()+10>virtual_state()->id()) return; //TODO: handle differently
			
			if(!current_active_state() && !already_set)
				return;
			
			//if already present in active list -> skip
			for(size_t i=0; i<active_states_.size(); i++)
				if(active_states_[i]==state) {
					if(!already_set) {
						//TODO: think about this correction
						//if(state->dist_trv()<0)
						//err	state->dist_trv() = distance_relation()/2;
						//state->dist_trv() = (1+state->dist_trv())/2;
						state->dist_dev() 	= std::min(state->dist_dev(), virtual_state()->dist_dev()+initial_distance());
						needs_sort_ = true;
					}
					return;
				}
				
			//somebody else will set this variables from outside
			if(!already_set) {
				state->dist_dev() 	= virtual_state()->dist_dev()+initial_distance();
				state->hops() 		= 0;
				state->reset_dist_trv();	//we are approaching state (assume half way)
				state->dist_trv_var()  	= 1;
			}
			
			//reset feature proability
			state->is_active() = true;
			state->reset_feature();
			
			//add to list
			active_states_.push_back(state);
			
			needs_sort_ = true;
		}		
		
		//!< remove state completely
		void remove_state(typename TState::TPtr &state) {
			if(!state) return;
			
			state->still_exists() = false;
			state->is_active() = false;
			state->hops() = 0;
			id_generator().register_removal(state);
			
			for(size_t i=0; i<active_states_.size(); i++)
				if(active_states_[i]==state)
					active_states_.erase(active_states_.begin()+i);
		}
		
		//!< remove unnecessary states from active state list and keep maximum size of list within limit
		void clean_active_list() {
			assert( sorting::is_sorted(active_states_.begin(),active_states_.end(), sorting::energy_order<typename TState::TPtr>) );
			
			//limit size of list
			if(active_states_.size()>param().max_active_states_) {
				for(size_t i=param().max_active_states_; i<active_states_.size(); i++)
					active_states_[i]->is_active() = false;
					
				active_states_.erase(active_states_.begin()+param().max_active_states_, active_states_.end());
			}
			
			//remove all departed states
			if(active_states_.size()>0) {
				//skip current active state
				for(size_t i=1; i<active_states_.size(); i++) {
					//if(i==0 && active_states_[i]==virtual_state()) continue;
					if(active_states_[i]==virtual_state()) continue;
					
					//if(active_states_[i]->dist_trv()-active_states_[i]->dist_trv_var()<=-1 || active_states_[i]->dist_dev()-active_states_.front()->dist_dev() > 10*initial_distance()) {
					if(active_states_[i]->dist_trv()<=-0.05f || active_states_[i]->dist_dev()-virtual_state()->dist_dev() > 10*initial_distance()) {
						active_states_[i]->is_active() = false;
						active_states_.erase(active_states_.begin()+i);
						--i;
					}
					
				}
			}
			
		}
		
		//getter/setter
		//!< settter/getter for active state list
		inline TActList &active_states() {return active_states_;}
		//!< getter for active state list
		inline const TActList &active_states() const {return active_states_;}
		
		//!< getter for parameters
		inline const TParameter &param() const {return param_;}
		//!< settter/getter for parameters
		inline TParameter &param_rw() {return param_;}
		
		inline typename TState::TPtr current_active_state() {return *active_states_.begin();} //TODO: check
		inline typename TState::TConstPtr current_active_state() const {return *active_states_.begin();} //TODO: check
		inline typename TState::TPtr &virtual_state() {return virtual_state_;}
		inline const typename TState::TConstPtr virtual_state() const {return virtual_state_;}
		inline typename TTransform::TPtr &virtual_transistion() {return virtual_transistion_;}
		inline const typename TTransform::TConstPtr virtual_transistion() const {return virtual_transistion_;}
		inline typename TState::TPtr &last_active_state() {return last_active_state_;}
		inline const typename TState::TConstPtr last_active_state() const {return last_active_state_;}
		
		inline const TEnergy &last_dist_min() const {return last_dist_min_;}
		inline TEnergy &last_dist_min() {return last_dist_min_;}
		
		//!< settter/getter for mutex
		boost::mutex &get_mutex() {return mtx_;}
		
		//!< if a feature was seen we connect the active state with the feature and update all connected states
		void add_feature(const typename TFeature::TID &id, const int ts, const typename TState::TFeatureClass &ft_class) {
			boost::lock_guard<boost::mutex> guard(mtx_);
			
			//if( current_active_state() && virtual_state() && current_active_state()->id() < virtual_state()->id()-(param().min_age_+3) )
			//	return;
			
			bool inject = true;
			for(typename FeatureBuffer::iterator it = last_features_.begin(); it!=last_features_.end(); it++)
				if(*it == id) {
					DBG_PRINTF("feature %d already set\n", id.id());
					inject = false;
					break;
				}
			last_features_.push_back(id);
			
			bool modified = false;
			typename FeatureMap::iterator it = features_.find(id);
			if(it==features_.end()) {
				it = features_.insert(typename FeatureMap::value_type(id, typename TFeature::TPtr(new TFeature(id))) ).first;
				modified = true;
			}
			//if( !(current_active_state() && virtual_state() && current_active_state()->id() < virtual_state()->id()-(param().min_age_+3) ) )
			if( current_active_state() && ft_slots_.size()>0 ) {
				if(current_active_state()==virtual_state()) {
					if(!ft_perceived_in(id, ft_slots_.front()))
						ft_slots_.front().push_back(id);
				}
				//else
				//	modified |= it->second->visited(current_active_state().get(), current_active_state());
			}
			
			if(it->second->inject(this, ts, param().est_occ_, param().max_active_states_, ft_class, inject))
				needs_sort_ = true;
			
			if(modified)
				id_generator().register_modification(it->second);
			id_generator().register_modification(current_active_state());
		}
		
		void merge_feature(const typename TFeature::TID &id, const typename TFeature::TPtr &ft) {
			typename FeatureMap::iterator it = features_.find(id);
			if(it==features_.end())
				it = features_.insert(typename FeatureMap::value_type(id, ft) ).first;
			else
				it->second->merge(ft);
		}

		UNIVERSAL_SERIALIZE()
		{
		    assert(version==CURRENT_SERIALIZATION_VERSION);;
		    
		    ar & UNIVERSAL_SERIALIZATION_NVP(param_);
		}
	};
	
	
	template<class TContext, class TGraph, class TMapStates, class TMapTransformations>
	class ContextContainer {
	protected:
		TContext *ctxt_;
		TGraph *graph_;
		TMapStates *states_;
		TMapTransformations *trans_;
		
	public:			
		ContextContainer() :
		 ctxt_(NULL), graph_(NULL), states_(NULL), trans_(NULL)
		 {}
		 
		ContextContainer(TContext *ctxt, TGraph *graph, TMapStates *states, TMapTransformations *trans) :
		 ctxt_(ctxt), graph_(graph), states_(states), trans_(trans)
		 {}
		    	
		UNIVERSAL_SERIALIZE()
		{
		    assert(version==CURRENT_SERIALIZATION_VERSION);
		    
		    //we have to lock everything for consistency
			boost::mutex mtx_tmp;
			boost::lock_guard<boost::mutex> guard(ctxt_ ? ctxt_->get_mutex() : mtx_tmp);
		    
			if(ctxt_)
			 ar & UNIVERSAL_SERIALIZATION_NVP_NAMED("ctxt", *ctxt_);
			else {
				TContext tmp;
				ar & UNIVERSAL_SERIALIZATION_NVP_NAMED("ctxt", tmp);
			}
		    
			std::vector<serialization::serializable_shared_ptr<typename TContext::TState> > states;
			std::vector<typename TContext::TState::TransitionSerialization> trans;
		    std::vector<typename TContext::TFeature::FeatureSerialization> fts;
			
			//on saving
		    if(graph_ && states_ && trans_ && UNIVERSAL_CHECK<Archive>::is_saving(ar)) {
				
				for(typename TGraph::NodeIt it(*graph_); it!=lemon::INVALID; ++it) {
					states.push_back((*states_)[it]);
					(*states_)[it]->get_trans(trans, *graph_, *states_, *trans_);
				}

				for(typename TContext::FeatureMap::iterator it = ctxt_->get_features().begin(); it!=ctxt_->get_features().end(); it++) {
					fts.push_back( it->second->get_serialization() );
				}
			}
		    
		    ar & UNIVERSAL_SERIALIZATION_NVP(fts);
			ar & UNIVERSAL_SERIALIZATION_NVP(states);
			ar & UNIVERSAL_SERIALIZATION_NVP(trans);
			
			//on loading
		    if(graph_ && states_ && trans_ && UNIVERSAL_CHECK<Archive>::is_loading(ar)) {
				//clear everything
				graph_->clear();
				ctxt_->active_states().clear();
				
				//insert states
				for(size_t i=0; i<states.size(); i++) {
					typename TContext::TState::TPtr c = states[i];
					
					c->set_node(graph_->addNode());
					states_->set(c->node(), c);
					//ctxt_.active_states().push_back(states[i]);
				}
				
				//insert transitions (TODO: speed up)
				for(typename TGraph::NodeIt it(*graph_); it!=lemon::INVALID; ++it)
					(*states_)[it]->set_trans(trans, *graph_, *states_, *trans_);
					
				//insert features
				for(size_t i=0; i<fts.size(); i++) {
					typename TContext::TFeature *tmp = new typename TContext::TFeature(fts[i].ft_.id());
					tmp->set_serialization(fts[i], *graph_, *states_, *trans_);
					
					ctxt_->get_features().insert(typename TContext::FeatureMap::value_type(fts[i].ft_.id(), typename TContext::TFeature::TPtr(tmp)) );
				}
				
			}
		}
			
	};
		
}
