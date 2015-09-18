
template<class TGraph, class TContext, class TMapStates, class TMapTransformations, class TTransformation>
void step(TGraph &graph, TContext &context, TMapStates &states, TMapTransformations &trans, const TTransformation &odom, const Eigen::Vector3f &dbg_pose) {
	typedef typename TContext::TState TState;
	typedef std::vector<typename TState::TPtr> TStateVector;

	path_integration<	TStateVector, typename TContext::TEnergyFactor,
						TGraph, TContext, TMapStates, TMapTransformations, TTransformation>
	(context.active_states(), graph, context, states, trans, odom, dbg_pose);
	
	context.clean_active_list();
}
