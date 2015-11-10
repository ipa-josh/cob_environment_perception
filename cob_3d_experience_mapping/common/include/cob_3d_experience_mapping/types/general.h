#pragma once

#include <lemon/list_graph.h>
#include <lemon/dijkstra.h>
#include <lemon/adaptors.h>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>

#include <ros/assert.h>


//! interfaces and implementations of cob_3d_experience_mapping
namespace cob_3d_experience_mapping {
	
	/**
	 * class: Empty
	 * parent class which is used by default if no meta data is needed for states (see Object)
	 */
	struct Empty {};
	
	template<class _TState, class _TFeature>
	class SimpleIdTsGenerator {
	public:
		typedef _TState TState;
		typedef _TFeature TFeature;
		typedef typename TState::ID ID;
		
	private:
		ID running_id_;
		/*
		 * std::map<ID, TModification>
		 */
		
	public:
		SimpleIdTsGenerator() : running_id_(1)
		{}
		
		ID new_id() {return running_id_++;}
		
		void register_modification(const typename TState::TPtr &state)
		{ }
		void register_modification(const typename TFeature::TPtr &state)
		{ }
		void register_removal(const typename TState::TPtr &state)
		{ }
	};
	
	/**
	 * class: Object
	 * parent class for a state to keep meta data like additonal map information (laser scans ...)
	 */
	template<class TMeta>
	class Object {
	protected:
		TMeta meta_;
	};
	
	struct DbgInfo {
		std::string name_;
		std::string info_;
		Eigen::Vector3f pose_;
		
		UNIVERSAL_SERIALIZE()
		{
		    ROS_ASSERT(version==CURRENT_SERIALIZATION_VERSION);
		    
		   ar & UNIVERSAL_SERIALIZATION_NVP(name_);
		   ar & UNIVERSAL_SERIALIZATION_NVP(info_);
		}
	};
	
	/**
	 * class: State
	 * short description: current state + activation --> Artificial Neuron
	 */
	template<class TMeta, class _TEnergy, class _TGraph, class _TLink, class _TID, class _TFeatureClass>
	class State : public Object<TMeta> {
	public:
		//types
		typedef _TEnergy TEnergy;
		typedef _TGraph TGraph;
		typedef typename TGraph::Node TNode;
		typedef typename TGraph::Arc TArc;
		typedef typename TGraph::OutArcIt TArcOutIterator;
		typedef typename TGraph::InArcIt TArcInIterator;
		typedef boost::shared_ptr<State> TPtr;
		typedef _TID ID;
		typedef _TFeatureClass TFeatureClass;
		typedef _TLink TLink;
		typedef std::map<TFeatureClass, uint32_t /*counter*/> TFeatureClassMap;
		
	protected:
		TEnergy dist_dev_, dist_trv_, dist_trv_var_, ft_imp_;
		TNode node_;
		TFeatureClassMap ft_class_occurences_;
		ID id_;
		int hops_;
		DbgInfo dbg_;		//!< some debug information like name and additional description (info)
		bool still_exists_;	//!< flag: false if removed from map completely
		bool is_active_;	//!< flag: true if present in active state list
		bool seen_;
		TLink trv_;
		
	public:		
		State(): dist_dev_(0), dist_trv_(0), dist_trv_var_(1), ft_imp_(1), id_(-1), hops_(0), still_exists_(true), is_active_(false), seen_(false) {
			dbg_.name_ = "INVALID";
		}
		
		State(const ID &id): dist_dev_(0), dist_trv_(0), dist_trv_var_(1), ft_imp_(1), id_(id), hops_(0), still_exists_(true), is_active_(false), seen_(false) {
			char buf[128];
			sprintf(buf, "%d", id_);
			dbg_.name_ = buf;
		}
		
		
		void update(const State &o) {
			ft_class_occurences_ = o.ft_class_occurences_;
			id_ = o.id_;
			dbg_ = o.dbg_;
		}
		
		//setter/getter		
		
		//!< getter for identifier
		inline ID  id() const {return id_;}
		
		//!< setter for identifier
		inline void set_id(const ID &id) {id_ = id;}

		//!< setter/getter for existance flag
		inline bool &still_exists() {return still_exists_;}
		//!< getter for existance flag
		inline bool  still_exists() const {return still_exists_;}

		//!< setter/getter for flag if in active state list
		inline bool &is_active() {return is_active_;}
		//!< getter for flag if in active state list
		inline bool  is_active() const {return is_active_;}
		
		//!< setter/getter for deviation distance
		inline TEnergy &dist_dev() {return dist_dev_;}
		//!< getter for deviation distance
		inline const TEnergy &dist_dev() const {return dist_dev_;}
		
		//!< setter/getter for travel distance
		//inline TEnergy &dist_trv() {return dist_trv_;}
		inline void set_dist_trv(const TEnergy &t) {dist_trv_=t;}
		//!< getter for travel distance
		inline const TEnergy &dist_trv() const {return dist_trv_;}
		
		inline void reset_dist_trv(const TLink &trv=TLink(), const TEnergy &dist=1) {
			dist_trv_ = dist;
			trv_ = trv;
		}
		
		inline const TLink &travel() const {
			return trv_;
		}
		
		inline const TLink &travel(const TLink &movement) {
			trv_.integrate(movement);
			return trv_;
		}
		
		inline void merge_trv(const bool overwrite, const TEnergy &pos, const TEnergy &var, const TLink &link) {
			if(overwrite) {
				DBG_PRINTF("state %d: merge OVERWRTIE    (%f/%f  %f/%f)\n", id(), pos, var, dist_trv_, dist_trv_var_);
				
				trv_.set_link( (1-pos)*link.get_data() );
				dist_trv_ 		= pos;
				dist_trv_var_ 	= var;
			}
			else {
				const TEnergy t	= (var*var*dist_trv_ + dist_trv_var_*dist_trv_var_*pos) / (var*var+dist_trv_var_*dist_trv_var_);
				
				DBG_PRINTF("state %d: merge %f    (%f/%f  %f/%f)\n", id(), t, pos, var, dist_trv_, dist_trv_var_);
				//assert(t+0.000001f>=dist_trv_);
				
				//if(std::abs(dist_trv_-1)>0.000001f)
				//	trv_       *= (1-t)/(1-dist_trv_);
				trv_.set_link( (var*var * trv_.get_data()  +  dist_trv_var_*dist_trv_var_ * (1-pos)*link.get_data())
							/ (var*var+dist_trv_var_*dist_trv_var_) );
				
				dist_trv_ 		= t;
				dist_trv_var_ 	= std::sqrt(1/(1/(dist_trv_var_*dist_trv_var_) + 1/(var*var)));
			}
		}
		
		//!< setter/getter for travel distance variance
		inline TEnergy &dist_trv_var() {return dist_trv_var_;}
		//!< getter for travel distance
		inline const TEnergy &dist_trv_var() const {return dist_trv_var_;}
		
		//!< setter/getter for hop counter
		inline bool &seen() {return seen_;}
		//!< getter for hop counter
		inline bool  seen() const {return seen_;}
		
		//!< setter/getter for hop counter
		inline int &hops() {return hops_;}
		//!< getter for hop counter
		inline int  hops() const {return hops_;}
		
		//!< setter for graph node
		inline void set_node(const TNode &node) {node_ = node;}
		
		//!< setter/getter for graph node
		inline TNode &node() {return node_;}
	
		//!< setter/getter for debug information
		inline DbgInfo &dbg() {return dbg_;}
		//!< getter for debug information
		inline const DbgInfo &dbg() const {return dbg_;}
		
		
		//graph operations
		//!< iterator operation for incoming transitions
		inline TArcInIterator arc_in_begin(const TGraph &graph) {
			return TArcInIterator(graph, node_);
		}
		//!< iterator operation for incoming transitions
		inline TArcInIterator arc_in_end(const TGraph &graph) const {
			return lemon::INVALID;
		}
		
		//!< iterator operation for outgoing transitions
		inline TArcOutIterator arc_out_begin(const TGraph &graph) {
			return TArcOutIterator(graph, node_);
		}
		//!< iterator operation for outgoing transitions
		inline TArcOutIterator arc_out_end(const TGraph &graph) const {
			return lemon::INVALID;
		}
		
		//!< iterator operation for transitions; e.g. for visualization, where it's not necessary only one direction
		template<typename _TArc_>
		inline _TArc_ arc_flex_begin(const TGraph &graph) {
			return _TArc_(graph, node_);
		}
		//!< iterator operation for transitions; e.g. for visualization, where it's not necessary only one direction
		template<typename _TArc_>
		inline _TArc_ arc_flex_end(const TGraph &graph) {
			return lemon::INVALID;
		}
		
		//!< getter to retrieve connected node of state by transition "ait" in "graph"
		inline TNode opposite_node(const TGraph &graph, const TArc &ait) {
			return graph.oppositeNode(node_, ait);
		}
		
		uint32_t get_feature_class_counter(const TFeatureClass &ft_cl) const {
			typename TFeatureClassMap::const_iterator it = ft_class_occurences_.find(ft_cl);
			if(it==ft_class_occurences_.end())
				return 0;
			return it->second;
		}
		
		//!< if a connected feature to this state was visited, we update the feature probability
		void update(const int ts, const int no_conn, const int est_occ, const uint32_t counter, const TFeatureClass &ft_cl, const TEnergy prob=1) {
			DBG_PRINTF("update for %d#%d: %d, %d", id(), (int)ft_class_occurences_.size(), ft_cl, counter);
			assert(ft_class_occurences_.find(ft_cl)!=ft_class_occurences_.end());
			DBG_PRINTF(" %d\n", ft_class_occurences_[ft_cl]);
			assert(counter<=ft_class_occurences_[ft_cl]);
					 
			ft_imp_ -= ft_imp_*prob*counter/((no_conn+est_occ)*ft_class_occurences_[ft_cl]);
			
			 //ft_imp_ *= 1-prob/(no_conn+est_occ);
			 //DBG_PRINTF("upd %d  %f\n", id(), get_feature_prob());
		}
		
		void visited_feature_class(const TFeatureClass &ft_cl) {
			typedef typename TFeatureClassMap::iterator I;
			std::pair<I,bool> const& r=ft_class_occurences_.insert(typename TFeatureClassMap::value_type(ft_cl,1));
			if (!r.second)
				r.first->second++;
			DBG_PRINTF("visited_feature_class for %d: %d\n", id(), r.first->second);
		}
		
		//!< getter for feature probability
		inline TEnergy get_feature_prob() const {return 1-ft_imp_;}
		
		//!< reset feature probability to default (no feature present)
		void reset_feature() {ft_imp_=1;seen_=false;}
		
		UNIVERSAL_SERIALIZE()
		{
		   assert(version==CURRENT_SERIALIZATION_VERSION);
		   
		   ar & UNIVERSAL_SERIALIZATION_NVP_NAMED("id", id_);
		   ar & UNIVERSAL_SERIALIZATION_NVP(ft_class_occurences_);
		   
		   if(UNIVERSAL_CHECK<Archive>::is_loading(ar))
			DBG_PRINTF("loaded %d with %d\n", (int)id_, (int)ft_class_occurences_.size());
		   
		   dbg_.serialize<Archive, make_nvp>(ar, version);
		}
		
		/**
		 * class: TransitionSerialization
		 * helper structure for serialization of all transitions of a state
		 */
		struct TransitionSerialization {
			ID src_, dst_;
			TLink link_;
			
			TransitionSerialization()
			{}
			
			TransitionSerialization(const ID &src, const ID &dst, const TLink &link) : src_(src), dst_(dst), link_(link)
			{}
			
			UNIVERSAL_SERIALIZE()
			{
				assert(version==CURRENT_SERIALIZATION_VERSION);
				
				ar & UNIVERSAL_SERIALIZATION_NVP_NAMED("src", src_);
				ar & UNIVERSAL_SERIALIZATION_NVP_NAMED("dst", dst_);
				ar & UNIVERSAL_SERIALIZATION_NVP(link_);
			}
		};
		
		template<class Graph, class TMapStates, class TMapTransformations>
		void get_trans(std::vector<TransitionSerialization> &trans_list, Graph &graph, TMapStates &states, TMapTransformations &trans)
		{
			if(!still_exists()) return;
			
			for(TArcOutIterator ait(arc_out_begin(graph)); ait!=arc_out_end(graph); ++ait) {
				if(states[opposite_node(graph, ait)]->still_exists())
					trans_list.push_back( TransitionSerialization(states[opposite_node(graph, ait)]->id(), id(), *trans[ait]) );
			}
		}
		
		template<class Graph, class TMapStates, class TMapTransformations>
		void set_trans(const std::vector<TransitionSerialization> &trans_list, Graph &graph, TMapStates &states, TMapTransformations &trans)
		{
			for(size_t i=0; i<trans_list.size(); i++) {
				if(trans_list[i].src_!=id()) continue;
				
				//check fo existing trans.
				bool found = false;
				for(TArcOutIterator ait(arc_out_begin(graph)); ait!=arc_out_end(graph); ++ait) {
					if(trans_list[i].dst_ == states[opposite_node(graph, ait)]->id()) {
						*trans[ait] = typename TMapTransformations::Value::element_type(trans_list[i].link_, trans[ait]->src(), trans[ait]->dst());
						found = true;
						break;
					}
				}
				if(found) continue;
				
				TPtr p_dst, p_src;
				for(typename TGraph::NodeIt it(graph); it!=lemon::INVALID && (!p_dst||!p_src); ++it) {
					if(states[it]->id_==trans_list[i].dst_)
						p_dst = states[it];
					if(states[it]->id_==id())
						p_src = states[it];
				}
				
				DBG_PRINTF("add arc %d %d\n", trans_list[i].src_, trans_list[i].dst_);
				
				assert(p_src);
				assert(p_dst);
				
				if(p_dst && p_dst->still_exists())
					trans.set(graph.addArc(p_dst->node(), node()), typename TMapTransformations::Value(
						new typename TMapTransformations::Value::element_type(trans_list[i].link_, p_src, p_dst)
					));
			}
		}
		
	};
	
	template<class _TFtID=int, class _TClID=int>
	struct FeatureID {
		typedef _TFtID TFtID;
		typedef _TClID TClID;
		
		TFtID id_;
		TClID class_;
		
		FeatureID(const TFtID &id) : id_(id), class_(-1) {}
		FeatureID(const TFtID &id, const TClID &cl) : id_(id), class_(cl) {}
		
		inline const TFtID &id() const {return id_;}
		inline const TClID &class_id() const {return class_;}
		
		inline bool operator<(const FeatureID &o) const {
			if(class_==o.class_)
				return id_<o.id_;
			return class_<o.class_;
		}
		
		inline bool operator==(const FeatureID &o) const {
			return id_==o.id_ && class_==o.class_;
		}
			
		UNIVERSAL_SERIALIZE()
		{
			assert(version==CURRENT_SERIALIZATION_VERSION);
			
			ar & UNIVERSAL_SERIALIZATION_NVP(id_);
			ar & UNIVERSAL_SERIALIZATION_NVP(class_);
		}
	};
	

	/**
	 * class: Feature
	 * short description: sensor input
	 */
	template<class TInjection, class TMeta, class _TID>
	class Feature : public Object<TMeta> {
	public:
		typedef _TID TID;
		typedef typename TInjection::TType TType;
		typedef boost::shared_ptr<Feature> TPtr;
		typedef void* StateHandle;
		typedef typename TInjection::TState::TFeatureClass TFeatureClass;
		
		struct Connection {
			typename TInjection::TPtr state_;
			uint32_t counter_;
			TType pos_, var_;
			
			Connection(typename TInjection::TPtr &s, uint32_t c, const TType &p, const TType &v):
				state_(s), counter_(c), pos_(p), var_(v)
			{}
			
			void merge(const Connection &o) {
				DBG_PRINTF("merge %d %d\n", state_->dst()->id(), o.state_->dst()->id());
				assert(state_ == o.state_);
				
				counter_ = o.counter_;	//TODO: this could remove already incremented counter between updates
			}
		};
		
		typedef std::map<StateHandle, Connection> InjectionMap;
		
	protected:
		TID id_;
		InjectionMap injections_;
		
	public:
		Feature(const TID &id) : id_(id)
		{}
		
		inline const TID &id() const {return id_;}
		
		void merge(const TPtr &o) {
			assert(id()==o->id());
			
			DBG_PRINTF("merge feature %d\n", id().id());
			
			for(typename InjectionMap::iterator it=o->injections_.begin(); it!=o->injections_.end(); it++) {
				typename InjectionMap::iterator itt = injections_.find(it->first);
				if(itt==injections_.end())
					injections_.insert(typename InjectionMap::value_type(it->first, it->second));
				else
					itt->second.merge(it->second);
			}
		}
		
		//!< setter for identifier
		inline void set_id(const TID &id) {id_ = id;}
		
		bool visited(const StateHandle &h, typename TInjection::TPtr inj, const TType &pos, const TType &var) {
			typename InjectionMap::iterator it = injections_.find(h);
			if(it==injections_.end()) {
				injections_.insert(typename InjectionMap::value_type(h, Connection(inj, 1, 1-pos, var)));
				
				//debug
				char buf[32];
				sprintf(buf,"ft%d ", id_.id());
				inj->dst()->dbg().info_ += buf;
				
				DBG_PRINTF("add feature %d -> %d with (%f/%f)\n", id_.id(), inj->dst()->id(), 1-pos, var);
				
				return true;
			}
			else {
				it->second.counter_++;

				DBG_PRINTF("check ft (%d) cnt: %d <= %d for %d\n", id_.id(), it->second.counter_, it->second.state_->dst()->get_feature_class_counter(0), it->second.state_->dst()->id());
				assert(it->second.counter_<=it->second.state_->dst()->get_feature_class_counter(0));
			}
			
			return false;
		}
		
		template<typename TContext>
		bool inject(TContext *ctxt, const int ts, const int est_occ, const int max_occ, const TFeatureClass &ft_cl, const bool update, const TType prob=1) {
			bool changed = false;
			for(typename InjectionMap::iterator it=injections_.begin(); it!=injections_.end(); it++) {
				//if state is already killed or still to be created --> skip injecting activation by external sensors
				if(!it->second.state_->dst()->still_exists() || it->second.state_->dst()==ctxt->virtual_state()) continue;
			
				//check if feature is in active list --> add if we not too ambiguous (50% of max. size of active state list)
				bool was_active = it->second.state_->dst()->is_active();
				if(2*injections_.size() < ctxt->param().max_active_states_)
					ctxt->add_to_active(it->second.state_->dst());
				it->second.state_->dst()->merge_trv(!was_active, it->second.pos_, it->second.var_, *it->second.state_);
				changed = true;
				
				if(update)
					it->second.state_->dst()->update(ts, std::min(max_occ, (int)injections_.size()), est_occ, it->second.counter_, ft_cl, prob);
				it->second.state_->dst()->seen() = true;
				
				DBG_PRINTF("injectXYZ %d -> %d with %d\n", id_.id(), it->second.state_->dst()->id(), (int)(injections_.size()+est_occ));
			}
			DBG_PRINTF("inject %d\n", (int)injections_.size());
			
			return changed;
		}
		
		/**
		 * class: FeatureSerialization
		 * helper structure for serialization of a feature
		 */
		struct FeatureSerialization {
			Feature ft_;
		    std::vector<typename TInjection::TState::ID> dsts_, srcs_;
		    std::vector<uint32_t> cnts_;
		    std::vector<TType> poss_, vars_;
			
			FeatureSerialization() : ft_(-1)
			{}
			
			FeatureSerialization(const Feature &ft) : ft_(ft)
			{}
			
			UNIVERSAL_SERIALIZE()
			{
				assert(version==CURRENT_SERIALIZATION_VERSION);
				
				ar & UNIVERSAL_SERIALIZATION_NVP(ft_);
				ar & UNIVERSAL_SERIALIZATION_NVP(dsts_);
				ar & UNIVERSAL_SERIALIZATION_NVP(srcs_);
				ar & UNIVERSAL_SERIALIZATION_NVP(cnts_);
				ar & UNIVERSAL_SERIALIZATION_NVP(poss_);
				ar & UNIVERSAL_SERIALIZATION_NVP(vars_);
			}
			
			void merge(const FeatureSerialization &o) {
				for(size_t j=0; j<o.dsts_.size(); j++) {
					bool found=false;
					for(size_t i=0; i<dsts_.size(); i++) {
						if(dsts_[i]!=o.dsts_[j]) continue;
						cnts_[i] = std::max(cnts_[i], o.cnts_[j]);
						found = true;
						break;
					}
					if(!found) {
						dsts_.push_back(o.dsts_[j]);
						srcs_.push_back(o.srcs_[j]);
						cnts_.push_back(o.cnts_[j]);
						poss_.push_back(o.poss_[j]);
						vars_.push_back(o.vars_[j]);
					}
				}
			}
		};
		
		UNIVERSAL_SERIALIZE()
		{
		    assert(version==CURRENT_SERIALIZATION_VERSION);
		    
		    ar & UNIVERSAL_SERIALIZATION_NVP(id_);
		}
		
		FeatureSerialization get_serialization() const {
			FeatureSerialization fs(*this);
			
			for(typename InjectionMap::const_iterator it=injections_.begin(); it!=injections_.end(); it++) {
				assert(it->second.state_);
				assert(it->second.state_->dst());
				assert(it->second.state_->src());
				
				if(!it->second.state_->dst()->still_exists() || !it->second.state_->src()->still_exists()) continue;
				
				fs.dsts_.push_back(it->second.state_->dst()->id());
				fs.srcs_.push_back(it->second.state_->src()->id());
				fs.cnts_.push_back(it->second.counter_);
				fs.poss_.push_back(it->second.pos_);
				fs.vars_.push_back(it->second.var_);
				
				DBG_PRINTF("send ft (%d) cnt: %d <= %d for %d/%d\n", id_.id(), fs.cnts_.back(), it->second.state_->dst()->get_feature_class_counter(0), fs.dsts_.back(), fs.srcs_.back());
				assert(fs.cnts_.back()<=it->second.state_->dst()->get_feature_class_counter(0));
			}
			
			return fs;
		}
		
		template<class TGraph, class TMapStates, class TMapTransformations>
		void set_serialization(const FeatureSerialization &fs, TGraph &graph, TMapStates &states, TMapTransformations &trans) {
			typedef typename TInjection::TState TState;
			typedef typename TState::TArcOutIterator TArcIter_out;
	
			id_ = fs.ft_.id();
			
			//reconnect features with states
			assert(fs.dsts_.size()==fs.srcs_.size());
			for(size_t i=0; i<fs.dsts_.size(); i++) {
				for(typename TGraph::NodeIt it(graph); it!=lemon::INVALID; ++it)
					if(states[it]->id()==fs.dsts_[i]) {
						states[it]->visited_feature_class(id_.class_id());
						
						DBG_PRINTF("check ft (%d) cnt: %d <= %d for %d/%d\n", id_.id(), fs.cnts_[i], states[it]->get_feature_class_counter(0), fs.dsts_[i], fs.srcs_[i]);
						assert(fs.cnts_[i]<=states[it]->get_feature_class_counter(0));
						
						bool exist=false;
						for(TArcIter_out ait(states[it]->arc_out_begin(graph)); ait!=states[it]->arc_out_end(graph); ++ait) {
							typename TState::TPtr opposite = states[states[it]->opposite_node(graph, ait)];
							//DBG_PRINTF("op id %d\n", opposite->id());
							if(opposite->id()==fs.srcs_[i]) {
								exist=true;
								injections_.insert(typename InjectionMap::value_type(trans[ait].get(), Connection(trans[ait], fs.cnts_[i], fs.poss_[i], fs.vars_[i]) ));
								break;
							}
						}
						
						assert(exist);				
						break;
					}
					
			}
		}
		
	};
}
