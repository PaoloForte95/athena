package org.pofe.athena.plan;


import java.io.FileNotFoundException;
import java.io.FileOutputStream;

import java.io.PrintStream;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.jgrapht.graph.DefaultDirectedGraph;
import org.jgrapht.graph.DefaultEdge;
import org.jgrapht.traverse.TopologicalOrderIterator;
import org.pofe.athena.util.Logging;

import fr.uga.pddl4j.parser.Connector;
import fr.uga.pddl4j.parser.Expression;

import org.pofe.athena.parser.Action;
import org.pofe.athena.verifier.PlanVerifier;



public class ConcurrentPlan extends PartialOrderPlan {

	protected HashMap <Action, ArrayList<Action>> concurrencySet=  new HashMap  <Action, ArrayList<Action>>();
	//Verifier
	protected PlanVerifier planVerifier;

	
	public ConcurrentPlan(PlanVerifier verifier) {
        super(verifier);
        logger = Logging.getLogger(ConcurrentPlan.class);
        actions =  new LinkedHashMap <Integer, ArrayList<Action>>();
        planVerifier = verifier;
        graphPlan =  new DefaultDirectedGraph<Action, DefaultEdge>(DefaultEdge.class);
	}
	
	public boolean add(int step, ArrayList<Action> actions) {
		if(this.actions.containsKey(step)) {
			ArrayList<Action> acts = this.actions.get(step);
			acts.addAll(actions);
			if(this.actions.replace(step, acts) != null) {
				return true;
			};
		}
		if(this.actions.put(step, actions)!= null) {
			return true;
		}
		
		return false;
	}
	
	@Override
	public boolean add(int step, Action action) {
		if(this.actions.containsKey(step)) {
			ArrayList<Action> actions = this.actions.get(step);
			actions.add(action);
			if(this.actions.replace(step, actions) != null) {
				return true;
			};
		}
		ArrayList<Action> actions = new ArrayList<Action>();
		actions.add(action);
		if(this.actions.put(step, actions)!= null) {
			return true;
		}
		
		return false;
	}



	@Override
	public boolean remove(int step, Action action) {
		if(this.actions.containsKey(step)) {
			ArrayList<Action> actions = this.actions.get(step);
			boolean removed = actions.remove(action);
			this.actions.replace(step, actions);
			return removed;
		}
		return false;
	}



	@Override
	public boolean remove(int step) {
		if(this.actions.containsKey(step)) {
			if(this.actions.remove(step) != null) {
				return true;
			};
		}
		return false;
	}



	@Override
	public boolean contains(int step, Action action) {
		if(this.actions.containsKey(step)) {
			for (Action act: this.actions.get(step)) {
				if(act.getName().contains(action.getName())) {
					return true;
				}
			}
			
		}
		return false;
	}



	@Override
	public boolean isEmpty() {
		return (this.actions.isEmpty());
	}



	@Override
	public Map<Integer, List<Action>> getActions() {
		HashMap<Integer, List<Action>> actions = new LinkedHashMap <Integer, List<Action>>();
		for (int step: this.actions.keySet() ) {
			actions.put(step,getActions(step));
		}
		return actions;
	}



	@Override
	public List<Action> getActions(int step) {
		ArrayList<Action> actions = new ArrayList<Action>();
		if(this.actions.containsKey(step)) {
			actions.addAll(this.actions.get(step));
			return actions;
		}
		return actions;
	}


	/**
	 * Create a Concurrent Plan and represent it with a directed graph. The plan is created by removing unnecessary constraints 
	 * between the actions from the original plan. Concurrent plan may be different to a Partial order plan. 
	 * In a concurrent plan, All actions must not be mutex (i.e., it is not possible to 
	 * have at the same step that an action produce an atom p and another action produce ¬p). 4 checks need to be performed:
	 * 1) If an action produces p, another action at the same step cannot produce ¬p
	 * 2) If an action produces ¬p, another action at the same step cannot produce p
	 * 3) If an action produces p, another action at the same step cannot produce p
	 * 4) If an action produces ¬p, another action at the same step cannot produce ¬p	
	 * @param graphPlan -> a graph representing the plan
	 * @return A concurrent plan represented with a directed graph
	 */
	@Override
	public DefaultDirectedGraph<Action, DefaultEdge> createGraph() {

			//See https://arxiv.org/pdf/1105.5441.pdf
			//Get the partial order plan 
			super.createGraph();


			//this.graphPlan = planGraph;
			TopologicalOrderIterator<Action, DefaultEdge> it = new TopologicalOrderIterator<Action, DefaultEdge>(graphPlan);

			//take the current mission and its parents
			while(it.hasNext()) {
				
				Action actualAction =  it.next();
				
				Set<DefaultEdge> previousEdges = new LinkedHashSet(graphPlan.incomingEdgesOf(actualAction));
				for(DefaultEdge edge : previousEdges) {
					Action previousAction = this.graphPlan.getEdgeSource(edge);
					//Check if the two actions are mutex or not
					boolean canBeConcurrent = concurrentActions(actualAction,previousAction);
					
					if(canBeConcurrent) {		
						logger.info("Action:" + actualAction + " and: " + previousAction +  " can be concurrent" );
						this.graphPlan.removeEdge(previousAction, actualAction);
						//The two actions could be executed concurrently
						//Save all the actions that could be executed concurrently with the current one
						/*
						if(!concurrencySet.containsKey(actualAction)) {
							ArrayList<Action> mutexActions = new ArrayList<Action>();
							mutexActions.add(previousAction);
							concurrencySet.put(actualAction, mutexActions);
						}
						else {
							ArrayList<Action> mutexActions = concurrencySet.get(actualAction);
							mutexActions.add(previousAction);
							concurrencySet.replace(actualAction, mutexActions);
						}
						*/
						//Get the action that have the precedence constraints with the previous one, and add the latest to the current action
						ArrayList<DefaultEdge> prevActionEdges = new ArrayList<DefaultEdge>(graphPlan.incomingEdgesOf(previousAction));
						if(prevActionEdges.size()>0) {
							Action lastPrevConstraint = graphPlan.getEdgeSource(prevActionEdges.get(0));
							this.graphPlan.addEdge(lastPrevConstraint, actualAction);
						}
		
					}
				
				 
				} 
			}
			//Save the plan in a file
			PrintStream fileStream = null;
			try {
				fileStream = new PrintStream((new FileOutputStream("ConcurrentPlan.txt",false)));	
			} catch (FileNotFoundException e) {

				e.printStackTrace();
			}
			//Print the plan
			TopologicalOrderIterator<Action, DefaultEdge> it2 = new TopologicalOrderIterator<Action, DefaultEdge>(graphPlan);
			while(it2.hasNext()) {
				Action m1 = it2.next();
				fileStream.append(m1.getID()+ ")" + m1.toString() + "\n");
				fileStream.append("Parents: ");
				for(DefaultEdge precedenceConstraint : graphPlan.incomingEdgesOf(m1)) {  
					Action parentMission = graphPlan.getEdgeSource(precedenceConstraint);
					fileStream.append(parentMission.toString() + "\n");
				 }
				fileStream.append("\n");
			}
			
		return this.graphPlan;	
		}
		
	
	/**
	 * Check if two actions are mutex. 4 checks need to be performed:
	 * 1) If an action produces p, the other action cannot produces ¬p
	 * 2) If an action produces ¬p, the other action cannot produce p
	 * 3) If an action produces p, the other action cannot produce p
	 * 4) If an action produces ¬p, the other action cannot produce ¬p	
	 * @param action1 -> a first action
	 * @param action2 -> a second action
	 * @return <code>true</code> if the actions are not mutex(i.e. can be executed concurrently), otherwise <code>false</code>
	 */
	protected boolean concurrentActions (Action action1, Action action2) {
		//Get the effects of the first action
		ArrayList<Expression<String>> effects = action1.getEffects();
		
		//Check if actions are mutex using preconditions -> works only with Temporal Planning 
		if(!checkTemporalPreconditions(action1.getPreconditions(),effects)) {
			return false;
		}
		
		for (Expression<String> effect : effects) {
			//Get the effects of the second action
			ArrayList<Expression<String>> prevEffects = action2.getEffects();
			for (Expression<String> prevEffect : prevEffects) {
				if(effect.getConnector()!= Connector.INCREASE) {
					
				}
				if(effect.getConnector() != Connector.NOT ) {	
					if(isNegated(effect,prevEffects)) {
						return false;
					}
					else if(isContained(effect,prevEffects)){
						return false;
					}
				}else {
					if(isProduced(effect,prevEffects)) {
						return false;
					}
					else if(isContained(effect,prevEffects)){
						return false;
						}
				}
	
				//only action for different robots can be concurrent
				// for(String param : action1.getParameters()) {
				// 	 ArrayList<String> paramType = this.problem.getType(param);
				// 	 if(!Collections.disjoint(paramType, this.problem.getTypeDef(this.problem.getRobotDef()))) {
				// 		 if(effect.toString().contains(action2.getInput(param).getVariable()) && prevEffect.toString().contains(action1.getInput(param).getVariable())) {
				// 				 return false;
				// 		 }
				// 	 }								
				// }
				for (String param : action1.getParameters()) {
					// Skip if action2 doesn't have this parameter
					if (action1.getInput(param) == null || action2.getInput(param) == null) {
        				continue;
					}	
					
					String variable1 = action1.getInput(param).getVariable();
					String variable2 = action2.getInput(param).getVariable();
					
					// If both effects involve variables of the same parameter type, actions cannot be concurrent
					if (effect.toString().contains(variable2) && prevEffect.toString().contains(variable1)) {
						return false;
					}
				}
			}
		}  
		return true;
	}
	
	/**
	 * Check between the temporal preconditions of the first action and temporal effects of the second one. 
	 * @param preconditions -> the precondition of the actual action
	 * @param effects -> effect of a previous action
	 * @return <code>true</code> if the actions are not mutex(i.e. can be executed concurrently), otherwise <code>false</code>
	 */

	protected boolean checkTemporalPreconditions(ArrayList<Expression<String>> preconditions, ArrayList<Expression<String>> effects){
		ArrayList <Boolean> preconditionsVerified = new ArrayList<Boolean>();
		boolean mutexActions = true;
			 for (Expression<String> precondition : preconditions) {
				 if(precondition.getConnector() == Connector.AT_START || precondition.getConnector() == Connector.AT_END ) {
					 Expression<String> prec = precondition.getChildren().get(0);	
					 boolean preconditionIsVerified = true;	  
						  for (Expression<String> effect : effects) {
							  Expression<String> eff = effect.getChildren().get(0);	
							  //It a precondition is atStart and it is contained into the effect of the other action, 
							  	if(prec.equals(eff)) {	
							  		switch (precondition.getConnector()) {
								  		case AT_START:
								  				preconditionIsVerified = false;
								  				break;
								  		case AT_END:
								  			//Only in this case the actions are not mutex
								  			if(effect.getConnector() == Connector.AT_START) {
								  				preconditionIsVerified = true;
								  				break;
								  			}else {
								  				preconditionIsVerified = false;
								  				break;
								  			}
										default:
											break;
							  		
							  		}
								  }							  							  
						  }
					  preconditionsVerified.add(preconditionIsVerified);
				 } //For precondition

				 for(boolean preconditionVerified : preconditionsVerified) {
					  if(!preconditionVerified) {
						  mutexActions = false;
					  }
					 
				  }
				 }
				 
		return mutexActions;
	}


	
	@Override
	public double makespan() {
		return this.actions.size();
	}


	@Override
	public double GetCost() {
		return this.actions.size();
	}
	
	public HashMap<Action, ArrayList<Action>> getConcurrencySet() {
		return this.concurrencySet;
	}
}
    
    

