package org.pofe.athena.verifier;

import java.io.*;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.logging.Logger;

import org.jgrapht.alg.TransitiveReduction;
import org.jgrapht.graph.DefaultDirectedGraph;
import org.jgrapht.graph.DefaultEdge;
import org.jgrapht.traverse.BreadthFirstIterator;
import org.jgrapht.traverse.TopologicalOrderIterator;
import fr.uga.pddl4j.parser.Connector;

import fr.uga.pddl4j.parser.Expression;


import org.pofe.athena.util.Logging;
import org.pofe.athena.parser.Action;
import org.pofe.athena.util.ExpressionUtil;
import org.pofe.athena.util.State;


/**
 * This class provides a simple verifier to check if the plan is feasible.
 * Perfomed checks:
 * 1) All paths exist
 * 
 * @author pofe
 *
 */
public class PlanVerifier {


		protected static Logger logger;
		protected List<Expression<String>> state;
		protected HashMap <String, Double> numerical_variables;
		
		public PlanVerifier(List<Expression<String>> init_state){
			this(init_state, new HashMap <String, Double>());
			
		}
		
		public PlanVerifier(List<Expression<String>> init_state, HashMap <String, Double>  numVariables){
			logger = Logging.getLogger(PlanVerifier.class);
			this.state = new ArrayList<Expression<String>>(init_state);
			this.numerical_variables = numVariables;
			
		}
		
		/**
		 * Check if an predicate is equal to another one. The predicates could also be numerical.
		 * @param predicate1 -> The first predicate
		 * @param predicate2 -> The second predicate
		 * @return -> <code>true</code> If the first predicate is equal to the second one, otherwise <code>false</code>
		 */
		public boolean comparePredicate (Expression<String> predicate1, Expression<String> predicate2) {
			boolean equal = false;
			double value1 = 0;
			double value2 = 0;
			
			//Temporal planning
			switch (predicate2.getConnector()) {
			case AT_START:
			case AT_END:
			case OVER_ALL:
				predicate2 = predicate2.getChildren().get(0);
				break;
			default:
				break;
			}
			
			switch (predicate1.getConnector()) {
			case AT_START:
			case AT_END:
			case OVER_ALL:
				predicate1 = predicate1.getChildren().get(0);
				break;
			default:
				break;
			}
		
			switch (predicate1.getConnector()) {
			    //Classical (Symbolic) Planning	
				case ATOM:
					if(predicate1.equals(predicate2)) {							
						 equal = true;
					 } 
					break;		
				case NOT:	
					Expression<String> negatedCondition = ExpressionUtil.negate(predicate2);
					if(predicate1.equals(negatedCondition)) {
						equal = true;
					}
					break;
				//Numerical Planning
				case LESS_COMPARISON:
				case LESS_OR_EQUAL_COMPARISON:
					if(this.numerical_variables.containsKey(predicate1.getChildren().get(0).toString())) {
						value1 = this.numerical_variables.get(predicate1.getChildren().get(0).toString());
						if(this.numerical_variables.containsKey(predicate1.getChildren().get(1).toString())) {
							value2 = this.numerical_variables.get(predicate1.getChildren().get(1).toString());
						}else {
							value2 = Double.parseDouble(predicate1.getChildren().get(1).toString());
						}
					}
					if(value1 <= value2) {
						 equal = true;
					}
					
					break;
				case EQUAL_COMPARISON:
					value1 = this.numerical_variables.get(predicate1.toString());
					value2 = this.numerical_variables.get(predicate2.toString());
					if(value1 == value2) {
						 equal = true;
					}
					break;
				case GREATER_COMPARISON:
				case GREATER_OR_EQUAL_COMPARISON:
					if(this.numerical_variables.containsKey(predicate1.getChildren().get(0).toString())) {
						value1 = this.numerical_variables.get(predicate1.getChildren().get(0).toString());
						if(this.numerical_variables.containsKey(predicate1.getChildren().get(1).toString())) {
							value2 = this.numerical_variables.get(predicate1.getChildren().get(1).toString());
						}else {
							value2 = Double.parseDouble(predicate1.getChildren().get(1).toString());
						}					
					}
					if(value1 >= value2) {
						 equal = true;
					}
					break;
				default:
					break;
			}

			return equal;
		}


		/**
		 * Check if a precondition/effect of an action is contained by a set of effects of a previous action or the current state.
		 * @param predicate -> a predicate of the current action 
		 * @param predicates -> set of effects related to a previous action in the plan 
		 * @return <code>true</code> if the expression is contained into the effects set of a previous action, otherwise <code>false</code>
		 */
		protected boolean isContained(Expression<String> predicate, ArrayList<Expression<String>> predicates) {
			boolean isContained = false;
			for (Expression<String> pred : predicates) {	
				if(pred.getChildren().size() > 0) {
					if(predicate.toString().equals(pred.toString())) {
						isContained = true;
						break;
					}
				}
			}
			return isContained;
		}

		/**
		 * Check if a predicate (effect or precondition) is negated by a set of effects of a previous action.
		 * @param predicate -> the predicate (precondition or effect) of the current action
		 * @param previousEffects -> Set of effects of a previous action in the plan 
		 * @return <code>true</code> if the predicate is negated by the previous action, otherwise <code>false</code>
		 */
	
		protected boolean isNegated(Expression<String> predicate, ArrayList<Expression<String>> previousEffects) {
			boolean isNegated = false;

			switch (predicate.getConnector()) {
			//Temporal Planning
			case AT_START:
			case AT_END:
			case OVER_ALL:
				Expression<String> negatedPredicate = ExpressionUtil.negate(predicate.getChildren().get(0));
				if(isContained(negatedPredicate,previousEffects)) {
					isNegated = true;
				}
				break;
				
			//Non Temporal Planning
			default:	
				negatedPredicate = ExpressionUtil.negate(predicate);
				if(isContained(negatedPredicate,previousEffects)) {
					isNegated = true;
				}
			break;
			
		
			}
			return isNegated;
		}
	
	
		/**
		 * Check if a predicate (effect or precondition) is produced by a set of effects of a previous action
		 * @param predicate -> The predicate (precondition or effect) of the current action
		 * @param previousEffects -> The set of effects of a previous action in the plan 
		 * @return <code>true</code> if the predicate is produced by the previous action, otherwise <code>false</code>
		 */
		
		protected boolean isProduced(Expression<String> predicate, ArrayList<Expression<String>> previousEffects) {
			boolean isProduced = false;
			switch (predicate.getConnector()) {
			case AT_START:
			case AT_END:
			case OVER_ALL:
				Expression<String> producedPredicate = ExpressionUtil.produce(predicate.getChildren().get(0));
				if(isContained(producedPredicate,previousEffects)) {
					isProduced = true;
				}
				break;		
			default:
				producedPredicate = ExpressionUtil.produce(predicate);
				if(isContained(producedPredicate,previousEffects)) {
					isProduced = true;
				}
				break;
		
			}
			return isProduced;
			
		}
		
		/**
		 * Check if the actual plan is valid. The first check verifies that all the actions preconditions are verified; the second check verifies that 
		 * between an action and its parent, no other actions negate a precondition
		 * @param plan -> the plan to verify expressed as a DefaultDirectedGraph
		 * @param init -> initial condition
		 * @return <code>true</code> if the plan is valid, otherwise <code>false</code>
		 */
	
		public boolean planValidation(DefaultDirectedGraph<Action, DefaultEdge> plan) {

			//logger.info("------------- PLAN VALIDATION -------------");
			//Initialize an iterator used to move through the graph of the plan
			TopologicalOrderIterator<Action, DefaultEdge> it= new TopologicalOrderIterator<Action, DefaultEdge>(plan);
			ArrayList <Action> actionSet = new ArrayList <Action>(plan.vertexSet()); 

			//Save for each action of the plan, if all the preconditions are verified.
			HashMap <Action, Boolean> planPreconditionsAreVerified =  new HashMap <Action, Boolean>();
			boolean planIsValid = true;
			
			//Start to move on the graph
			while(it.hasNext()) {
				planIsValid = true;	
				//Get a mission
				Action actualAction =  it.next();
				//Get the precedence constraints related to the current action (i.e. all the actions that need to be performed before the 
				//current action.
				ArrayList <DefaultEdge> precedenceConstraints = new ArrayList <DefaultEdge>(plan.incomingEdgesOf(actualAction));
				//Save the index of the current action and of its parent (i.e. the action that must be performed before the actual mission to have 
				//a valid plan)
				int indexActualAction = actionSet.indexOf(actualAction);
				int indexPreviousAction = indexActualAction;
				ArrayList <Boolean> preconditionsAreVerified = new ArrayList<Boolean>();
				ArrayList<Expression<String>> preconditions = actualAction.getPreconditions();
				//Check if all the preconditions of the current action are still verified by the actual plan (i.e. the plan is still valid)
				boolean allPreconditionsValid = true;				
				for (Expression<String> precondition : preconditions) {
					//logger.info("Checking Precondition: " + precondition);
						boolean preconditionVerified = false;
						
						//If this mission has some parents 
						if(precedenceConstraints.size() > 0) {
							//Check if a precondition of the current action is generated/negated by an effect of a previous action
							//If this criteria is not valid for one precondition, the actual plan is not valid
							for(int index = (indexActualAction-1); index >= 0 ; index--) {
								Action previousAction = actionSet.get(index);
								if(plan.containsEdge(previousAction, actualAction)){
									ArrayList<Expression<String>> previousEffects = previousAction.getEffects();
									for (Expression<String> effect : previousEffects) {
										if(comparePredicate(precondition,effect)) {
											//The current predicate is still generated by the effect of a previous action!
											preconditionVerified = true;
											//Save the index of the action that verified this predicate
											indexPreviousAction = actionSet.indexOf(previousAction);
											break;
										}
									}
									//Stop at the first required parent
									if(preconditionVerified) {
										break;
									}	
								}  
							}
							//Check also in the initial conditions
							if(!preconditionVerified) {
								if (precondition.getConnector() == Connector.AT_START || precondition.getConnector() == Connector.AT_END 
										 || precondition.getConnector() == Connector.OVER_ALL) {
									if(this.state.contains(precondition.getChildren().get(0))) {
										preconditionVerified = true;
										indexPreviousAction = 0;	
									}	
								}else {
									if(this.state.contains(precondition)) {
										  preconditionVerified = true;
										  indexPreviousAction = 0;
									}else if(!this.state.contains(precondition) && precondition.getConnector() == Connector.NOT ) {
											 preconditionVerified = true;
											 indexPreviousAction = 0;
									}
								}
							}
							
							
						}
						//If this mission does not have a parent, check on initial condition
						else {
		
							if (precondition.getConnector() == Connector.AT_START || precondition.getConnector() == Connector.AT_END 
									 || precondition.getConnector() == Connector.OVER_ALL) {
								 if(this.state.contains(precondition.getChildren().get(0))) {
									 preconditionVerified = true;
								 }else {
									 for(Expression<String> init : this.state) {
											if(comparePredicate(precondition,init)) {	
												preconditionVerified = true;
												
											} 
									 }
								 }
							}else {
								//First check: Symbolic precondition
								if(this.state.contains(precondition)) {
									preconditionVerified = true;
									indexPreviousAction = 0;
									
								}
								//Second check: for a negative precondition, check that the initial state does not have that variable initialize (i.e.
								//in PDDL this means that the variable is initialize as false)
								if(!this.state.contains(precondition) && precondition.getConnector() == Connector.NOT ) {
									preconditionVerified = true;
									indexPreviousAction = 0;
								}
							}
						}
						
						
						/**
						 * Check for the second condition. Given the steps of the current action and of the action that verifies the current precondition
						 * (i.e. produce or consume it), no other actions effects between these two steps must negate the precondition
						 */
						
						Action parentMission = actionSet.get(indexPreviousAction);
						for(Action action: actionSet ) {
							ArrayList<Expression<String>> previousEffects = action.getEffects();
							//Check if some actions between the two steps negate the actual precondition
						
							if(plan.containsEdge(action, actualAction) && actionSet.indexOf(action) >= indexPreviousAction) {
								if(isNegated(precondition,previousEffects)) {							
									preconditionVerified = false;
						
									break;
								}
							}
							else if(parentMission.equals(action) && !parentMission.equals(actualAction) ) {
								if(isNegated(precondition,previousEffects)) {							
									preconditionVerified = false;
						
									break;
								}
							
							}
						
							else if(actionSet.indexOf(action) == 0 && precedenceConstraints.size() == 0 && indexActualAction > 0 ) {
								if(isNegated(precondition,previousEffects)) {	
									  preconditionVerified = false;
						
									  break;
								}
							}

						}
					if(!preconditionVerified) {
						logger.info("Precondition " + precondition + " of action " + actualAction + " is NOT VERIFIED.");
					}
					preconditionsAreVerified.add(preconditionVerified);
				 } 
				 for(boolean preconditionIsVerified : preconditionsAreVerified) {
					  if(!preconditionIsVerified) {
						  allPreconditionsValid = false;
					  }
					 
				 }
				 planPreconditionsAreVerified.put(actualAction, allPreconditionsValid);
			} //End while mission 
			
			for(Action m: planPreconditionsAreVerified.keySet() ) {
				if(!planPreconditionsAreVerified.get(m)) {
					planIsValid = false;
					break;
				}
			}
			return planIsValid;
		}

		/**
		 * Check if the actual plan is valid. The first check verifies that all the actions preconditions are verified; the second check verifies that 
		 * at the same step:
		 * 1) If an action produces p, another action at the same step produces ¬p
		 * 2)  If an action produces ¬p, another action at the same step produces p
		 * 3)  If an action produces p, another action at the same step produces p
		 * 4)  If an action produces ¬p, another action at the same step produces ¬p
		 * @param plan -> the graph of the plan to verify
		 * @param init -> initial condition
		 * @return <code>true</code> if the plan is valid, otherwise <code>false</code>
		 * @throws IOException
		 */
		public boolean StatePlanValidation(DefaultDirectedGraph<Action,DefaultEdge> plan) throws IOException {
			//Initialize the state
			State state = new State();
			//Add the initial conditions to the state
			ArrayList<Expression<String>> initialState = new ArrayList<Expression<String>>();
			initialState.addAll(this.state);
			state.updateState(initialState);
			//Reduce the current graph 
			DefaultDirectedGraph<Action, DefaultEdge> directedGraphReduced = (DefaultDirectedGraph<Action, DefaultEdge>) plan.clone();
			TransitiveReduction.INSTANCE.reduce(directedGraphReduced);
			//logger.info("------------- PLAN VALIDATION -------------");
			//Initialize a operator that will be used only to get the depth of an action
			BreadthFirstIterator<Action, DefaultEdge> it = new BreadthFirstIterator<Action, DefaultEdge>(directedGraphReduced);
			//To get the depth of an action, firstly the iterator needs to cross all the graph.
			while(it.hasNext()) {
				it.next();	  
			}
			HashMap <Action, Boolean> planPreconditionsAreVerified =  new HashMap <Action, Boolean>();
			boolean planIsValid = true;
			//Initialize a operator that will go through the plan
			TopologicalOrderIterator<Action, DefaultEdge> it2 = new TopologicalOrderIterator<Action, DefaultEdge>(directedGraphReduced); 
			
			int previousActionDepth = 0;
			int currentActionDepth = 0;
			ArrayList <Expression<String>> newState =  new ArrayList <Expression<String>>();
			while(it2.hasNext()) {
				 planIsValid = true;
				 Action actualAction =  it2.next();
				 currentActionDepth = it.getDepth(actualAction);
				 //If the iterator moves to the next "level" of the graph, update the state by applying the effects of the actions at the previous level
				 if(currentActionDepth != previousActionDepth) {
					 state.updateState(newState);
					 newState.removeAll(newState);
				 }
				 ArrayList<Expression<String>> currentState = state.getCurrentState();
				 //logger.info("------------- ACTION -------------");
				 //logger.info("Action " + actualAction);	
				 //logger.info("Action depth on the graph " + it.getDepth(actualAction));	
				  
				 ArrayList<Expression<String>> preconditions = actualAction.getPreconditions();
				 ArrayList<Expression<String>> effects = actualAction.getEffects();
				 
				 ArrayList <Boolean> preconditionsAreVerified = new ArrayList<Boolean>();
			 
				 newState.addAll(effects);
				 //logger.info("Current State " + currentState );
				 boolean allPreconditionsValid = true;
				 boolean preconditionVerified = false;
				//Check for first condition
				 for (Expression<String> precondition : preconditions) {
					 //logger.info("Precondition " + precondition);
					 preconditionVerified = false;
					 for(Expression<String> atom : currentState) {								 
						 if(comparePredicate(precondition,atom)) {
							 preconditionVerified = true;
						  }
					 }
					 //Check in the initial state
					 if(currentActionDepth == 0) {
						 //for a negative precondition, check that the initial state does not have that variable initialize (i.e.
						 //in PDDL this means that the variable is initialize as false)
						 if(!this.state.contains(precondition) && precondition.getConnector() == Connector.NOT ) {
							 preconditionVerified = true;
						 }	 
					 }
					 
					  preconditionsAreVerified.add(preconditionVerified);
				 } // End first check condition
				 //Check for second condition 
				 TopologicalOrderIterator<Action, DefaultEdge> it3= new TopologicalOrderIterator<Action, DefaultEdge>(directedGraphReduced); 
				 //Take the effects of all the actions at the same depth
				 ArrayList <Expression<String>> concurrentEffects =  new ArrayList <Expression<String>>();
				 int actionDepth = currentActionDepth;
				 while(currentActionDepth == actionDepth && it3.hasNext()) {
					 Action concurrentAction =  it3.next();
					 actionDepth = it.getDepth(concurrentAction);
					 if(actualAction != concurrentAction) {
						 concurrentEffects.addAll(concurrentAction.getEffects());
					 }
	 
				 }
				 preconditionVerified = checkConcurrentEffects(effects,concurrentEffects);
				 preconditionsAreVerified.add(preconditionVerified);	 
				 for(boolean preconditionIsVerified : preconditionsAreVerified) {
					 //logger.info("Precondition is verified: " + preconditionIsVerified);
					  if(!preconditionIsVerified) {
						  allPreconditionsValid = false;
					  }
					 
				 }
				 planPreconditionsAreVerified.put(actualAction, allPreconditionsValid);
				 //Take the depth of the current Mission
				 previousActionDepth = it.getDepth(actualAction);
			}//All the mission have been analyzed 
			for(Action m: planPreconditionsAreVerified.keySet() ) {
				if(!planPreconditionsAreVerified.get(m)) {
					planIsValid = false;
					break;
				}
			}
			return planIsValid;
		}

		/**
		 * Verifies that: 
		 * 1) If an effect produces p, another effect at the same step does not produce ¬p
		 * 2) If an effect produces ¬p, another effect at the same step does not produce p
		 * 3) If an effect produces p, another effect at the same step does not produce p
		 * 4) If an effect produces ¬p, another effect at the same step does not produce ¬p
		 * @param effects -> the effect of an action 
		 * @param concurrentEffects -> the set of effects of the actions at same depth in the graph
		 * @return
		 */
		
		protected boolean checkConcurrentEffects(ArrayList<Expression<String>> effects, ArrayList<Expression<String>> concurrentState) {
			boolean preconditionVerified = true;
			

			for(Expression<String> effect: effects ) {
				if(effect.getConnector() != Connector.NOT) {	
					if(isNegated(effect,concurrentState)) {
						preconditionVerified = false;
						break;
					}
					else if(isContained(effect,concurrentState)){
						preconditionVerified = false;
						break;
					}
				}else {
					if(isProduced(effect,concurrentState)) {
						preconditionVerified = false;
						break;
					}
					else if(isContained(effect,concurrentState)){
						preconditionVerified = false;
						break;
					}
				}	
			}
			return preconditionVerified;
		}
			

}