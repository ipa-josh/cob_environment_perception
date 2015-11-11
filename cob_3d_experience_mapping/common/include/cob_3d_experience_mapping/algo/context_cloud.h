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
	
	template<class _TState, class _TFeature>
	class ClientIdTsGenerator {
	public:
		typedef _TState TState;
		typedef _TFeature TFeature;
		typedef typename TState::ID ID;
		typedef typename TFeature::TID FtID;
		
	private:
		ID running_id_;
		
		std::map<ID, typename TState::TPtr>   modification_states_;
		std::map<FtID, typename TFeature::TPtr> modification_fts_;
		
	public:
		ClientIdTsGenerator() : running_id_(1)
		{}
		
		ID new_id() {return running_id_++;}
		
		void register_modification(const typename TState::TPtr &state)
		{
			assert(state);
			modification_states_[state->id()] = state;
		}
		void register_modification(const typename TFeature::TPtr &ft)
		{
			assert(ft);
			modification_fts_[ft->id()] = ft;
		}
		void register_removal(const typename TState::TPtr &state)
		{
			assert(state);
			modification_states_[state->id()] = state;
		}
		
		void clear() {
			modification_states_.clear();
			modification_fts_.clear();
		}
		
		void get_lists(std::vector<serialization::serializable_shared_ptr<TState> > &updated_states, std::vector<ID> &removed_states, std::vector<typename TFeature::TPtr> &updated_fts)
		{
			for(typename std::map<ID, typename TState::TPtr>::iterator it = modification_states_.begin(); it!=modification_states_.end(); it++) {
				//if( it->second->still_exists() )
					updated_states.push_back( serialization::serializable_shared_ptr<TState>(it->second) );
				if( !it->second->still_exists() )
					removed_states.push_back( it->first );
				DBG_PRINTF("upload state %d %d\n", it->second->id(), (int)it->second->still_exists() );
			}
			
			for(typename std::map<FtID, typename TFeature::TPtr>::iterator it = modification_fts_.begin(); it!=modification_fts_.end(); it++)
				updated_fts.push_back( it->second );
		}
		
	};
	
	template<class TContext, class TGraph, class TMapStates, class TMapTransformations, class _TClientId, class TArchiveIn = boost::archive::binary_iarchive, class TArchiveOut = boost::archive::binary_oarchive>
	class IncrementalContextContainer : public ContextContainer<TContext, TGraph, TMapStates, TMapTransformations> {
		
	public:
		typedef _TClientId TClientId;
		typedef typename TContext::TState TState;
		typedef typename TContext::TFeature TFeature;
		typedef typename TState::ID ID;
		typedef serialization::NetworkHeader<TClientId, hiberlite::sqlid_t> TNetworkHeader;
		
	private:
		TNetworkHeader net_header_;
#ifdef SERVER_
		boost::shared_ptr<hiberlite::Database> server_;
#endif
		
		std::vector<serialization::serializable_shared_ptr<TState> > copy_updated_states_;
		std::vector<typename TContext::TFeature::FeatureSerialization> copy_fts_;
		std::vector<typename TContext::TState::TransitionSerialization> copy_trans_;
		
	public:
		
		IncrementalContextContainer() :
		 ContextContainer<TContext, TGraph, TMapStates, TMapTransformations>()
		 {}
		 
		IncrementalContextContainer(TContext *ctxt, TGraph *graph, TMapStates *states, TMapTransformations *trans, const TClientId &client_id=(TClientId)-1) :
		 ContextContainer<TContext, TGraph, TMapStates, TMapTransformations>(ctxt, graph, states, trans)
		{
			 //if(ctxt) ctxt->id_generator().set_client_id(client_id);
		}
		
		TNetworkHeader get_network_header() const {
			return net_header_;
		}
		
		void set_network_header(const TNetworkHeader &nh) {
			net_header_ = nh;
		}
		
		void upload(const char * addr, const char * port, const int timeout_secs=120) {
			serialization::sync_content_client<TArchiveIn, TArchiveOut> (*this, addr, port, timeout_secs);
		}
		
#ifdef SERVER_
		typedef std::map<ID, ID> TMapID_ID;
		typedef std::map<TClientId, TMapID_ID > TMapClientID_ID;
		//typedef std::map<typename TFeature::TID, sqlid_t> TMapFtID_DbID;
		
		TMapClientID_ID id_conv_map_2server, id_conv_map_client;
		//hiberlite::bean_ptr<TMapFtID_DbID> id_conv_ft_;
		
		ID convert2client_id(const ID id) const {
			typename TMapClientID_ID::const_iterator itc = id_conv_map_client.find(net_header_.client_);
			if(itc==id_conv_map_client.end()) return id;
			typename TMapID_ID::const_iterator it = itc->second.find(id);
			if(it==itc->second.end()) return id;
			return it->second;
		}
		
		ID convert2server_id(const ID id) const {
			if(id<0) return id;
			
			typename TMapClientID_ID::const_iterator itc = id_conv_map_2server.find(net_header_.client_);
			assert(itc!=id_conv_map_2server.end());
			typename TMapID_ID::const_iterator it = itc->second.find(id);
			assert(it!=itc->second.end());
			return it->second;
		}
		
		void on_client(std::iostream &stream, TClientId &running_client_id) {
			clock_t begin = clock();
			
			if(!server_) {
				server_.reset( new hiberlite::Database("sample.db") );
				
				server_->registerBeanClass<TState>();
				server_->registerBeanClass<typename TContext::TState::TransitionSerialization>();
				server_->registerBeanClass<typename TContext::TFeature::FeatureSerialization>();
				//server_->registerBeanClass<TMapFtID_DbID>();
				
				server_->createModel();
			}
			
			DBG_PRINTF("took_1 %f\n", double(clock() - begin) / CLOCKS_PER_SEC);
			
			serialization::sync_content_server_import_header<TArchiveIn, TArchiveOut> (*this, stream);
			
			if(net_header_.client_==(TClientId)-1) {
				net_header_.client_ = running_client_id;
				
				DBG_PRINTF("got new client, setting id to %d\n", net_header_.client_);
			}
			
			//update next client id
			running_client_id = std::max(running_client_id, (TClientId)(net_header_.client_+1));
			
			copy_updated_states_.clear();
			copy_fts_.clear();
			copy_trans_.clear();
			
			DBG_PRINTF("took_2 %f\n", double(clock() - begin) / CLOCKS_PER_SEC);
			
			std::vector< hiberlite::bean_ptr<TState> > v_s = server_->getAllBeanAfter<TState>(net_header_.ts_states_);
			for(size_t i=0; i<v_s.size(); i++) {
				copy_updated_states_.push_back( serialization::serializable_shared_ptr<TState>(v_s[i].shared_ptr()));
				copy_updated_states_.back()->set_id( convert2client_id(copy_updated_states_.back()->id()) );
				net_header_.ts_states_ = std::max(v_s[i].get_id(), net_header_.ts_states_);
			}
			
			std::vector< hiberlite::bean_ptr<typename TContext::TFeature::FeatureSerialization> > v_f = server_->getAllBeanAfter<typename TContext::TFeature::FeatureSerialization>(net_header_.ts_fts_);
			for(size_t i=0; i<v_f.size(); i++) {
				copy_fts_.push_back(*v_f[i]);
				net_header_.ts_fts_ = std::max(v_f[i].get_id(), net_header_.ts_fts_);
				v_f[i].shared_ptr(); //prevent db update
				
				DBG_PRINTF("sending ft %d\n", copy_fts_.back().ft_.id().id());
				
				for(size_t j=0; j<copy_fts_.back().dsts_.size(); j++) {
					assert(copy_fts_.back().dsts_[j]<0);
					assert(copy_fts_.back().srcs_[j]<0);
					DBG_PRINTF("sending dsts_ %d -> %d\n", copy_fts_.back().dsts_[j], convert2client_id(copy_fts_.back().dsts_[j]));
					DBG_PRINTF("sending srcs_ %d -> %d\n", copy_fts_.back().srcs_[j], convert2client_id(copy_fts_.back().srcs_[j]));
					copy_fts_.back().dsts_[j] = convert2client_id(copy_fts_.back().dsts_[j]);
					copy_fts_.back().srcs_[j] = convert2client_id(copy_fts_.back().srcs_[j]);
				}
			}
			
			std::vector< hiberlite::bean_ptr<typename TContext::TState::TransitionSerialization> > v_t = server_->getAllBeanAfter<typename TContext::TState::TransitionSerialization>(net_header_.ts_trans_);
			for(size_t i=0; i<v_t.size(); i++) {
				copy_trans_.push_back(*v_t[i]);
				net_header_.ts_trans_ = std::max(v_t[i].get_id(), net_header_.ts_trans_);
				v_t[i].shared_ptr(); //prevent db update
				
				copy_trans_.back().dst_ = convert2client_id(copy_trans_.back().dst_);
				copy_trans_.back().src_ = convert2client_id(copy_trans_.back().src_);
			}
			
			DBG_PRINTF("took_3 %f\n", double(clock() - begin) / CLOCKS_PER_SEC);
				
			serialization::sync_content_server_import<TArchiveIn, TArchiveOut> (*this, stream);
			
			DBG_PRINTF("took_4 %f\n", double(clock() - begin) / CLOCKS_PER_SEC);
			
			serialization::sync_content_server_export<TArchiveIn, TArchiveOut> (*this, stream);
			
			DBG_PRINTF("took_5 %f\n", double(clock() - begin) / CLOCKS_PER_SEC);
			
			//ok, now set the loaded content to the list and export it
			copy_updated_states_.clear();
			copy_fts_.clear();
			copy_trans_.clear();
		}
#endif
		  
		UNIVERSAL_SERIALIZE()
		{
			clock_t begin = clock();
			
			std::vector<serialization::serializable_shared_ptr<TState> > updated_states;
			std::vector<ID> removed_states;
			std::vector<typename TContext::TState::TransitionSerialization> trans;
		    std::vector<typename TContext::TFeature::FeatureSerialization> fts;
		    
			DBG_PRINTF("took_A1 %f\n", double(clock() - begin) / CLOCKS_PER_SEC);
			
			if(this->ctxt_ && this->graph_ && this->states_ && this->trans_ && UNIVERSAL_CHECK<Archive>::is_saving(ar)) {
#ifdef SERVER_
					get_lists_server(updated_states, fts, trans, removed_states);
#else
					get_lists_client(updated_states, fts, trans, removed_states);
#endif
			}
		    
			DBG_PRINTF("took_A2 %f\n", double(clock() - begin) / CLOCKS_PER_SEC);
			
		    ar & UNIVERSAL_SERIALIZATION_NVP(fts);
		    ar & UNIVERSAL_SERIALIZATION_NVP(updated_states);
		    ar & UNIVERSAL_SERIALIZATION_NVP(removed_states);
		    ar & UNIVERSAL_SERIALIZATION_NVP(trans);
		    
			DBG_PRINTF("took_A3 %f\n", double(clock() - begin) / CLOCKS_PER_SEC);
			DBG_PRINTF("serialize size %d %d %d\n", (int)updated_states.size(), (int)trans.size(), (int)removed_states.size());
		    
			if(this->ctxt_ && this->graph_ && this->states_ && this->trans_ && UNIVERSAL_CHECK<Archive>::is_loading(ar)) {
#ifdef SERVER_
				if(server_)
					on_loaded_server(updated_states, fts, trans, removed_states);
#else
					on_loaded_client(updated_states, fts, trans, removed_states);
#endif
			}
		    
			DBG_PRINTF("took_A4 %f\n", double(clock() - begin) / CLOCKS_PER_SEC);
			
		}
		
		virtual void get_lists_server(
			std::vector<serialization::serializable_shared_ptr<TState> > &updated_states,
			std::vector<typename TContext::TFeature::FeatureSerialization> &fts, 
			std::vector<typename TContext::TState::TransitionSerialization> &trans,
			std::vector<ID> &removed_states
		) {
			updated_states = copy_updated_states_;
			fts = copy_fts_;
			trans = copy_trans_;
			removed_states.clear();
		}
		
		virtual void get_lists_client(
			std::vector<serialization::serializable_shared_ptr<TState> > &updated_states,
			std::vector<typename TContext::TFeature::FeatureSerialization> &fts, 
			std::vector<typename TContext::TState::TransitionSerialization> &trans,
			std::vector<ID> &removed_states
		) {
		    //we have to lock everything for consistency
			boost::mutex mtx_tmp;
			boost::lock_guard<boost::mutex> guard(this->ctxt_ ? this->ctxt_->get_mutex() : mtx_tmp);
			
			std::vector<typename TFeature::TPtr> updated_fts;
			
			this->ctxt_->id_generator().get_lists(updated_states, removed_states, updated_fts);
			
			for(size_t i=0; i<updated_states.size(); i++)
				updated_states[i]->get_trans(trans, *this->graph_, *this->states_, *this->trans_);

			for(size_t i=0; i<updated_fts.size(); i++)
				fts.push_back( updated_fts[i]->get_serialization() );
				
			this->ctxt_->id_generator().clear();
		}
		
#ifdef SERVER_
		virtual void on_loaded_server(
			std::vector<serialization::serializable_shared_ptr<TState> > &updated_states,
			std::vector<typename TContext::TFeature::FeatureSerialization> &fts, 
			std::vector<typename TContext::TState::TransitionSerialization> &trans,
			const std::vector<ID> &removed_states
		) {
			assert(server_);
			
			//remove old or modified entries
			//TODO: do this
			
			for(size_t j=0; j<updated_states.size(); j++)
				updated_states[j]->set_id( convert2client_id(updated_states[j]->id()) );
			
			server_->begin_transaction();
			for(size_t j=0; j<updated_states.size(); j++) {
				if(updated_states[j]->id()>=0) continue;
				DBG_PRINTF("checking %d to remove from db\n", updated_states[j]->id());
				hiberlite::bean_ptr<TState> p = server_->loadBean<TState>(-updated_states[j]->id());
				if(!p || p.operator->()==NULL) continue;
				
				for(size_t k=0; k<copy_updated_states_.size(); k++)
					if(copy_updated_states_[k]->id()==updated_states[j]->id())
						assert(0);//*copy_updated_states_[k] = *updated_states[j];
				
				DBG_PRINTF("removed old state %d from db\n", p->id());
				p.destroy();	//remove old one
				DBG_PRINTF("destroyed\n");
			}
			DBG_PRINTF("done\n");
			
			//save to db	
			for(size_t i=0; i<updated_states.size(); i++) {
				hiberlite::bean_ptr<TState> p=server_->copyBean(*updated_states[i]);
				
				if(updated_states[i]->id()>0) {
					id_conv_map_2server[net_header_.client_][updated_states[i]->id()] = -(ID)p.get_id();
					id_conv_map_client[net_header_.client_][-(ID)p.get_id()] = updated_states[i]->id();
					p->set_id( -(ID)p.get_id() );
				}
				
				DBG_PRINTF("new state %d with %d\n", p->id(), p->get_feature_class_counter(0));
				net_header_.ts_states_ = std::max(p.get_id(), net_header_.ts_states_);
			}
			
			for(size_t i=0; i<fts.size(); i++) {
				DBG_PRINTF("new ftA %d\n", fts[i].ft_.id().id());
				for(size_t j=0; j<fts[i].dsts_.size(); j++) {
					DBG_PRINTF("\tstateA %d / %d: %d\n", fts[i].dsts_[j], convert2server_id(fts[i].dsts_[j]), fts[i].dsts_[j]);
					if(fts[i].dsts_[j]>0) fts[i].dsts_[j] = convert2server_id(fts[i].dsts_[j]);
					DBG_PRINTF("\tstateA %d / %d: %d\n", fts[i].srcs_[j], convert2server_id(fts[i].srcs_[j]), fts[i].srcs_[j]);
					if(fts[i].srcs_[j]>0) fts[i].srcs_[j] = convert2server_id(fts[i].srcs_[j]);
				}
			}
			
			std::vector< hiberlite::bean_ptr<typename TContext::TFeature::FeatureSerialization> > v_f = server_->getAllBeans<typename TContext::TFeature::FeatureSerialization>();
			for(size_t i=0; i<v_f.size(); i++) {
				//DBG_PRINTF("ft %d %d\n", (int)i, (int)(v_f[i].operator->()!=NULL));
				if(v_f[i].operator->()==NULL) continue;
				
				bool found = false;
				for(size_t j=0; j<fts.size(); j++) {
					if(v_f[i]->ft_.id()==fts[j].ft_.id()) {
						DBG_PRINTF("removed old feature %d from db\n", v_f[i]->ft_.id().id());
						v_f[i].destroy();	//remove old one
						found = true;
						
						for(size_t k=0; k<copy_fts_.size(); k++)
							if(copy_fts_[k].ft_.id()==fts[j].ft_.id())
								copy_fts_[k].merge(fts[j]);
						break;
					}
				}
				if(!found) v_f[i].shared_ptr(); //prevent db update
			}
			
			for(size_t i=0; i<trans.size(); i++) {
				if(trans[i].src_>0) trans[i].src_ = convert2server_id(trans[i].src_);
				if(trans[i].dst_>0) trans[i].dst_ = convert2server_id(trans[i].dst_);
				DBG_PRINTF("new trans %d %d\n", trans[i].src_, trans[i].dst_);
				hiberlite::bean_ptr<typename TContext::TState::TransitionSerialization> p=server_->copyBean(trans[i]);
				net_header_.ts_trans_ = std::max(p.get_id(), net_header_.ts_trans_);
			}
			
			for(size_t i=0; i<fts.size(); i++) {
				DBG_PRINTF("new ft %d\n", fts[i].ft_.id().id());
				for(size_t j=0; j<fts[i].dsts_.size(); j++) {
					DBG_PRINTF("\tstate %d / %d: %d\n", fts[i].dsts_[j], convert2server_id(fts[i].dsts_[j]), fts[i].cnts_[j]);
					if(fts[i].dsts_[j]>0) fts[i].dsts_[j] = convert2server_id(fts[i].dsts_[j]);
					DBG_PRINTF("\tstate %d / %d: %d\n", fts[i].srcs_[j], convert2server_id(fts[i].srcs_[j]), fts[i].cnts_[j]);
					if(fts[i].srcs_[j]>0) fts[i].srcs_[j] = convert2server_id(fts[i].srcs_[j]);
				}
				hiberlite::bean_ptr<typename TContext::TFeature::FeatureSerialization> p=server_->copyBean(fts[i]);
				net_header_.ts_fts_ = std::max(p.get_id(), net_header_.ts_fts_);
			}
			server_->commit_transaction();
		}
#endif
		
		virtual void on_loaded_client(
			const std::vector<serialization::serializable_shared_ptr<TState> > &updated_states,
			const std::vector<typename TContext::TFeature::FeatureSerialization> &fts, 
			const std::vector<typename TContext::TState::TransitionSerialization> &trans,
			const std::vector<ID> &removed_states
		) {
		    //we have to lock everything for consistency
			boost::mutex mtx_tmp;
			boost::lock_guard<boost::mutex> guard(this->ctxt_ ? this->ctxt_->get_mutex() : mtx_tmp);
			
			//insert states
			for(size_t i=0; i<updated_states.size(); i++) {
				DBG_PRINTF("load state %d", (int)updated_states[i]->id());
				bool found = false;
				for(typename TGraph::NodeIt it(*this->graph_); it!=lemon::INVALID; ++it) {
					if( (*this->states_)[it]->id() == updated_states[i]->id() ) {
						DBG_PRINTF(": updated\n");
						(*this->states_)[it]->update( *updated_states[i] );
						found = true;
						break;
					}
				}
					
				if(found) continue;
				DBG_PRINTF("\n");
				
				typename TContext::TState::TPtr c(updated_states[i]);
				
				c->set_node(this->graph_->addNode());
				this->states_->set(c->node(), c);
			}
			
			for(size_t i=0; i<removed_states.size(); i++) {
				bool found = false;
				for(typename TGraph::NodeIt it(*this->graph_); it!=lemon::INVALID; ++it) {
					if( (*this->states_)[it]->id() == removed_states[i] ) {
						this->ctxt_->remove_state( (*this->states_)[it] );
						found = true;
						break;
					}
				}
				
				if(!found)
					DBG_PRINTF_URGENT("WARNING: could not find state to be removed\n");
			}
			
			//insert transitions (TODO: speed up)
			for(typename TGraph::NodeIt it(*this->graph_); it!=lemon::INVALID; ++it)
				(*this->states_)[it]->set_trans(trans, *this->graph_, *this->states_, *this->trans_);
			
			//insert features
			for(size_t i=0; i<fts.size(); i++) {
				typename TContext::TFeature *tmp = new typename TContext::TFeature(fts[i].ft_.id());
				tmp->set_serialization(fts[i], *this->graph_, *this->states_, *this->trans_);
				
				this->ctxt_->merge_feature(fts[i].ft_.id(), typename TContext::TFeature::TPtr(tmp));
			}
		}
		
	};
	
}
