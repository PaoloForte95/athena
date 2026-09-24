package org.pofe.athena.util;

import fr.uga.pddl4j.parser.Connector;
import fr.uga.pddl4j.parser.Expression;

/**
 * Provide functions to modify a PDDL Epxression
 * @author pofe
 *
 */

public class ExpressionUtil{

	/**
	 * Negate a predicate.
	 * @param exp -> The predicate to negate
	 * @return negation -> the negation of the predicate give as input
	 */
	public static Expression<String> negate(Expression<String> expression) {
		if(expression.getConnector().equals(Connector.NOT)) {
			//If expression is already a negation, make it a producer
			return produce(expression);
		}
		//create the same predicate as before but negated
		Expression<String> negation = new Expression<String>(Connector.NOT); 
		negation.addChild(expression);

		return negation;
	}
	
	/**
	 * "Produce" (Make positive) a predicate 
	 * @param exp -> the predicate to produce 
	 * @return producer -> the producer of the predicate give as input
	 */
	
	public static Expression<String> produce(Expression<String> expression) {
		Expression<String> predicate = expression.getChildren().get(0);
		Expression<String>  producer = new Expression<String>(Connector.ATOM);
		producer.setArguments(predicate.getArguments());
		producer.setSymbol(predicate.getSymbol());
		if(predicate.getVariable() != null) {
			producer.setValue(predicate.getValue());
		}
		for(Expression<String> child: predicate.getChildren()) {
			producer.addChild(child);
		}
		
	    return producer;  
	}
	
	
	
}



