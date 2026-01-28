package org.pofe.athena.util;

import java.util.ArrayList;
import java.util.List;

import fr.uga.pddl4j.parser.Connector;
import fr.uga.pddl4j.parser.Expression;


public class State {
	


	protected ArrayList <Expression<String>> currentState =  new ArrayList <Expression<String>>();

	
	
	
	public ArrayList<Expression<String>> getCurrentState(){
		return this.currentState;
	}
	

	public void updateState(List<Expression<String>> state) {
		
		for(Expression<String> actualState : state ) {
			//An atom that was true is now false
			if(actualState.getConnector() == Connector.NOT) {
				
				Expression<String> producerState = ExpressionUtil.produce(actualState);
				if(currentState.contains(producerState)) {
					currentState.remove(producerState);
					currentState.add(actualState);
				}else {
					currentState.add(actualState);
				}
			}else {
				//An atom that was false is now true
				//PDDLExpression producerState = myExpression.producer(actualState);
				Expression<String> negatedState = ExpressionUtil.negate(actualState);
				if(currentState.contains(negatedState)) {
					currentState.remove(negatedState);
					currentState.add(actualState);
				}else {
					currentState.add(actualState);
				}
			}	
			
		}
	}
	
	
	@Override
	public String toString() {
		return currentState.toString();
	}


}



