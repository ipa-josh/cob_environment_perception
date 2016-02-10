#pragma once

#include <cmath>
#include <stdio.h>
#include <iostream>
#include <Eigen/Core>

//#define NDEBUG

#ifdef NDEBUG

#define DBG_PRINTF(str, ...) ;
#define ERROR_PRINTF(str, ...) ROS_ERROR(str, ##__VA_ARGS__)

#define DBG_PRINTF_URGENT(str, ...) ;
//#define DBG_PRINTF_URGENT(str, ...) {printf(str, ##__VA_ARGS__);fflush(stdout);}

#else

#define DBG_PRINTF(str, ...) {printf(str, ##__VA_ARGS__);fflush(stdout);}
#define ERROR_PRINTF(str, ...) ROS_ERROR(str, ##__VA_ARGS__)
#define DBG_PRINTF_URGENT(str, ...) {printf(str, ##__VA_ARGS__);fflush(stdout);}

#endif

namespace cob_3d_experience_mapping {
	
	//! debugging functions for logging
	namespace debug {
		
		template<class V, class T>
		bool _dbg_cmp_le_rad(const V &a, const V &b, const T t) {
			for(int i=0; i<a.rows(); i++) {
				if( M_PI - std::fabs(std::fmod(std::fabs(a(i) - b(i)), 2*M_PI) - M_PI) > t )
					return false;
			}
			return true;
		}

		template<class TContext, class TPose>
		void print_active_list(const TContext &ctxt, const TPose &dbg_pose) {
			typedef typename TContext::TActList::const_iterator TIter;
			typedef typename TContext::TState TState;
			
			TIter begin = ctxt.active_states().begin();
			TIter end   = ctxt.active_states().end();

			int i=0;
			DBG_PRINTF("current_list");
			for(TIter it=begin; it!=end; it++) {
				bool gt = (dbg_pose.template head<2>()-(*it)->dbg().pose_.template head<2>()).norm()<ctxt.param().debugging_prox_thr_(0);
				gt &= _dbg_cmp_le_rad<Eigen::Matrix<typename TState::TEnergy,1,1>, typename TState::TEnergy>(dbg_pose.template tail<1>(),(*it)->dbg().pose_.template tail<1>(),ctxt.param().debugging_prox_thr_(1));
				
				if(i==0)
					DBG_PRINTF("%c\n",gt?'o':'*');
				
				bool gti = ctxt.virtual_state() && (*it)->id() < ctxt.virtual_state()->id()-10;
				gti &= (dbg_pose.template head<2>()-(*it)->dbg().pose_.template head<2>()).norm()<2*ctxt.param().debugging_prox_thr_(0);
				gti &= _dbg_cmp_le_rad<Eigen::Matrix<typename TState::TEnergy,1,1>, typename TState::TEnergy>(dbg_pose.template tail<1>(),(*it)->dbg().pose_.template tail<1>(),2*ctxt.param().debugging_prox_thr_(1));
				DBG_PRINTF("%d:\t %f:%f:%f:%d   %s %s\n", (*it)->id(), (*it)->dist_dev(), (*it)->dist_trv(), (*it)->dist_trv_var(), (*it)->hops(),
					gt?"MATCH_GT ":" ", gti?"INTEREST":""
				);
				++i;
				//if(i>3) break;
			}
			DBG_PRINTF("\n");
			
			DBG_PRINTF("current id %d\n", ctxt.current_active_state()->id());
		}
		
		template<class TContext, class Type>
		void print_changing_dist_V(const TContext &ctxt, const Type &dev_increment) {
			DBG_PRINTF("%d changing dist V (%f/%f) by (%f, %f)\n", ctxt.virtual_state()->id(), ctxt.virtual_state()->dist_trv(), ctxt.virtual_state()->dist_dev(), dev_increment, (1-ctxt.ft_current_slot_similiarity()));
		}
		
		template<class TContext>
		void print_resetting_virtual_state(const TContext &ctxt) {
			if(ctxt.virtual_state()) {
				ctxt.virtual_transistion()->dbg();
			}
		
			DBG_PRINTF("resetting virtual state %d (%f)\n",
				(int)(ctxt.last_active_state()!=ctxt.current_active_state()),
				ctxt.virtual_state()?ctxt.virtual_state()->dist_trv():0.
			);
		}
		
		template<class TContext>
		void print_insert_virtual_state(const TContext &ctxt) {
			DBG_PRINTF("virtual state is inserted to map (action dist.: %f)\n", ctxt.virtual_transistion()->dist());
			
			DBG_PRINTF("dev vec %f \t%f\n", ctxt.virtual_transistion()->deviation()(0), ctxt.virtual_transistion()->deviation()(2));
			DBG_PRINTF("tr  vec %f \t%f\n", ctxt.virtual_transistion()->get_data()(0), ctxt.virtual_transistion()->get_data()(2));
		}

		template<class TContext>
		void print_relocalized(const TContext &ctxt) {
			DBG_PRINTF_URGENT("relocalized %d -> %d with %d hops\n", ctxt.last_active_state()?ctxt.last_active_state()->id():-1, ctxt.current_active_state()?ctxt.current_active_state()->id():-1,
				ctxt.current_active_state()?ctxt.current_active_state()->hops():0);
		}

		template<class TContext, class TPose>
		void print_relocalized(const TContext &ctxt, const bool exist, const TPose &dbg_pose) {
			typedef typename TContext::TState TState;
			
			if(exist)
				{DBG_PRINTF("not inserted new link as exists already\n");}
			else
				{DBG_PRINTF("inserted new link %d -> %d", ctxt.virtual_transistion()->src()->id(), ctxt.current_active_state()->id());}
				
			std::cout<<"old pose: "<<ctxt.last_active_state()->dbg().pose_.transpose()<<std::endl;
			std::cout<<"new pose: "<<ctxt.current_active_state()->dbg().pose_.transpose()<<std::endl;
			std::cout<<"cur pose: "<<dbg_pose.transpose()<<std::endl;
			
			bool gt = (dbg_pose.template head<2>()-ctxt.current_active_state()->dbg().pose_.template head<2>()).norm()<=ctxt.param().debugging_prox_thr_(0)*2;
			gt &= _dbg_cmp_le_rad<Eigen::Matrix<typename TState::TEnergy,1,1>, typename TState::TEnergy>(dbg_pose.template tail<1>(),ctxt.current_active_state()->dbg().pose_.template tail<1>(), ctxt.param().debugging_prox_thr_(1)*2);
			
			DBG_PRINTF_URGENT("pose match %f %f "
				, (dbg_pose.template head<2>()-ctxt.current_active_state()->dbg().pose_.template head<2>()).norm() * 40008000 / 360 
				, (ctxt.last_active_state()->dbg().pose_.template head<2>()-ctxt.current_active_state()->dbg().pose_.template head<2>()).norm() * 40008000 / 360 );
			DBG_PRINTF_URGENT( (dbg_pose.template head<2>()-ctxt.current_active_state()->dbg().pose_.template head<2>()).norm() * 40008000 / 360<=(dbg_pose(2)+ctxt.current_active_state()->dbg().pose_(2))?
				"SUCCESS1  ":"FAILED1  "
			);
			DBG_PRINTF_URGENT( (ctxt.last_active_state()->dbg().pose_.template head<2>()-ctxt.current_active_state()->dbg().pose_.template head<2>()).norm() * 40008000 / 360<=(ctxt.last_active_state()->dbg().pose_(2)+ctxt.current_active_state()->dbg().pose_(2))?
				"SUCCESS2 ":"FAILED2 "
			);
			DBG_PRINTF_URGENT( gt ? "SUCCESS3\n":"FAILED3\n"	);
		}
	
		template<class TState, class Type>
		void print_ft_prob_before(const TState &state, const Type &ft_prob) {
			if(ft_prob)
				DBG_PRINTF("%d: injecting energy %f (feature probability)    %f  %d hops  %f %f\n", state.id(), ft_prob, state.get_feature_prob(), state.hops(), state.dist_dev(), state.dist_dev());
		}
		
		template<class TState, class Type>
		void print_ft_prob_after(const TState &state, const Type &ft_prob) {
			if(ft_prob)
				DBG_PRINTF("%d: new energy %f\n", state.id(), state.dist_dev());
		}
		
#if 0
		
		template<class TState, class TTrans>
		void print_travel(const TState &state, const TState &oppposite, const TTrans &trans_best /*trans[best]*/) {
			std::cout<<"dbg travel\n"<<state->travel().get_data().transpose()<<"\n"<<trans_best->directed(state).get_data().transpose()<<"\n"<<opposite->travel().get_data().transpose()<<std::endl;
		}		
				
				DBG_PRINTF("%d changing dist(%f/%f) %f, %f, %f with R=%f\n\n", (*it)->id(),
					(*it)->dist_trv(),(*it)->dist_dev(), dev_min, dev_increment, sim_best, R);
					DBG_PRINTF("1 %d:%d setting dist %f/%f with %d hops (off %f)\n\n",
						opposite->id(), (*it)->id(),
						opposite->dist_trv(), opposite->dist_dev(),
						opposite->hops(), off
						);
#endif	

	}
}
