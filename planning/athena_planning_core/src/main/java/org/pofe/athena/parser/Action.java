package org.pofe.athena.parser;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;

import fr.uga.pddl4j.parser.Expression;
import fr.uga.pddl4j.parser.Symbol;
import fr.uga.pddl4j.parser.SymbolType;

/**
 * This class create a instance of an action. Each action is characterized by sets of inputs, parameters, precondition and effects that are automatically imported from 
 * the problem and domain files.
 * @author pofe
 *
 */
public class Action {
	
	protected String name;
	protected int ID;
	protected ArrayList <Expression<String>> preconditions =  new ArrayList <Expression<String>>();
	protected ArrayList <Expression<String>> effects =  new ArrayList <Expression<String>>();
	protected ArrayList <String> parameters = new ArrayList <String>();
	protected LinkedHashMap<String, SymbolicSymbol> inputs =  new LinkedHashMap <String, SymbolicSymbol>();
	protected Expression<String> duration;
	
	/**
	 * Create an action with an empty name.
	 */
	public Action() {
		this("", 1);
	}
	
	/**
	 * Create an action with a specific name.
	 * @param name -> the action name
	 */
	
	public Action(String name) {
		this(name, 1);
	}

	/**
	 * Create an action with a specific name and a specific ID.
	 * @param name -> the action name
	 * @param ID -> the action ID
	 */
	
	 public Action(String name,int ID) {
		this.name = name;
		this.ID = ID;
	}
	
	/**
	 * Set the name of the action 
	 * @param name -> the name to give to the action
	 */
	public void setName(String name) {
		this.name = name;
	}

	/**
	 * Set the ID of the action 
	 * @param ID -> the ID to associate to the action
	 */
	public void setID(int ID) {
		this.ID = ID;
	}

	/**
	 * Get the action ID
	 * @param ID -> the ID associate to the action
	 */
	public int getID() {
		return this.ID;
	}

	/**
	 * Get the action name
	 * @return actionName -> the name of the action
	 */
	public String getName () {
		return this.name;
	}
	
	/**
	 * Add a set of parameters to the action
	 * @param parameters -> set of parameters to add
	 */
	public void addParameters (ArrayList <String> parameters) {
		this.parameters.addAll(parameters);
	}
	
	/**
	 * Add a parameter to the action
	 * @param parameter > parameter to add
	 */
	public void addParameter ( String parameter) {
		this.parameters.add(parameter);
	}
	
	/**
	 * Get the set of all parameters of the action
	 * @return The set of all action parameters
	 */
	public ArrayList<String> getParameters(){
		return this.parameters; 
	}
	
	
	/**
	 * Check if the action has a specific parameter
	 * @param param -> parameter to check 
	 * @return -> <code>true</code> if the action has the parameter, otherwise <code>false</code>
	 */
	public boolean hasParameter(String param) {
		return (this.inputs.containsKey(param));
	}
	
	/**
	 * Add a set of preconditions to the action
	 * @param preconditions ->  set of preconditions to add
	 */
	public void addPreconditions (ArrayList<Expression<String>> preconditions) {
		this.preconditions.addAll(preconditions);
	}
	
	/**
	 * Add a precondition to the action
	 * @param precondition -> precondition to add
	 */
	public void addPrecondition (Expression<String> precondition) {
		this.preconditions.add(precondition);
	}
	
	/**
	 * Add a set of effects to the action
	 * @param effects ->  set of effects to add
	 */
	public void addEffects (ArrayList<Expression<String>> effects) {
		this.effects.addAll(effects);
	}
	
	
	/**
	 * Add an effect to the action
	 * @param precondition -> effect to add
	 */public void addEffect (Expression<String> effect) {
		this.effects.add(effect);
	}

	 /**
	  * Add an input (i.e. an object) to the action, and associate it with a parameter. 
	  * @param param -> parameter name 
	  * @param input -> input name
	  */
	public void addInput (String param, SymbolicSymbol input) {
		this.inputs.put(param, input);
		
	}
	
	

	
	/**
	 * Get the set of all inputs of the actions
	 * @return The set of all actions inputs
	 */
	public ArrayList <SymbolicSymbol> getInputs() {
		ArrayList<SymbolicSymbol> inputs = new ArrayList<SymbolicSymbol>();
		for(String param: this.inputs.keySet()) {
			inputs.add(this.inputs.get(param));
		}
		return inputs;
		
	}
	
	
	/**
	 * Get the input associated to a specific parameter in this action
	 * @param param -> parameter name
	 * @return the input associated to the specific parameter, otherwise null
	 */
	public SymbolicSymbol getInput(String param) {
		if(this.inputs.containsKey(param)) {
			return this.inputs.get(param);
		}
		return null;
		
	}
	
	
	private static boolean isNumeric(String str) { 
		  try {  
		    Double.parseDouble(str);  
		    return true;
		  } catch(NumberFormatException e){  
		    return false;  
		  }  
		}
	
	private void substituteArguments(Expression<String> expression){
		List<Symbol<String>> arguments = new ArrayList<Symbol<String>>(expression.getArguments());
		for(Symbol<String> i : arguments) {				
			SymbolicSymbol j = this.inputs.get(i.getImage());
			if(j != null) {
				int index = expression.getArguments().indexOf(i);
				expression.getArguments().remove(i);
				Symbol<String> symbol = new Symbol<String>(SymbolType.VARIABLE, j.getVariable());
				expression.getArguments().add(index, symbol);
			}
		}	
	}


	/**
	 * Get a precondition where each parameter of the precondition has been substituted with the corresponding input.
	 * @return The action precondition
	 */
	
	private Expression<String> getPrecondition(Expression<String> precondition){
		Expression<String> copy = new Expression<String>(precondition);
		switch (precondition.getConnector()) {
		//Symbolic Precondition (positive)
		case ATOM:
			substituteArguments(copy);
			break;
		//Symbolic Precondition (negative)
		case NOT:
			substituteArguments(copy.getChildren().get(0));
			break;	
		//Numerical Precondition
		case LESS_COMPARISON:
		case LESS_OR_EQUAL_COMPARISON:
		case EQUAL_COMPARISON:
		case GREATER_COMPARISON:
		case GREATER_OR_EQUAL_COMPARISON:
		case FN_ATOM:
			substituteArguments(copy.getChildren().get(0).getChildren().get(0));
			/////////////////
			if(isNumeric(copy.getChildren().get(1).toString())) {
				copy.setValue(Double.parseDouble(copy.getChildren().get(1).toString()));
			}
			///////////////////
			
			Double local1 = copy.getChildren().get(1).getChildren().get(0).getValue();
			if(local1 != null) {
				substituteArguments(copy.getChildren().get(1).getChildren().get(0));
			}
			else{
				substituteArguments(copy.getChildren().get(1).getChildren().get(0).getChildren().get(0));
			}		
			break;	
		default:
			break;
	}
		return copy;
	}
	
	/**
	 * Get the set of action preconditions where each parameter has been substituted with the corresponding input.
	 * @return The set of action preconditions
	 */
	
	public ArrayList<Expression<String>> getPreconditions() {
		//Create a copy of the actions preconditions set, so the real set will be not changed
		ArrayList <Expression<String>> 	preconditions = new ArrayList <Expression<String>>();
		for(Expression<String> precondition: this.preconditions) {
			Expression<String> copy = new Expression<String>(precondition);
			switch (precondition.getConnector()) {
			//Temporal Precondition
			case AT_START:
			case AT_END:
			case OVER_ALL:
				copy = getPrecondition(precondition.getChildren().get(0));				
				break;
			//Numerical or Symbolic Precondition
			default:
				copy = getPrecondition(precondition);
				break;
			}
			preconditions.add(copy);	
		}	
		 return preconditions; 
	}
	
	
	/**
	 * Get a precondition where each parameter of the effect has been substituted with the corresponding input.
	 * @return The action effect
	 */
	
	private Expression<String> getEffect(Expression<String> effect){
			Expression<String> copy = new Expression<String>(effect);
			switch (effect.getConnector()) {	
			//Symbolic Positive Effect
			case ATOM:
				substituteArguments(copy);
				break;
			//Symbolic Negative Effect
			case NOT:
				substituteArguments(copy.getChildren().get(0));
				break;
			case INCREASE:
			case DECREASE:
			case LESS_OR_EQUAL_COMPARISON:
			case EQUAL_COMPARISON:
			case GREATER_COMPARISON:
			case GREATER_OR_EQUAL_COMPARISON:
			case ASSIGN:
				substituteArguments(copy.getChildren().get(0));
				
				Double copy1 = copy.getChildren().get(1).getChildren().get(0).getValue();
				if(copy1 != null) {
					substituteArguments(copy.getChildren().get(1).getChildren().get(0));
				}
				//for Expression with operation 
				else{
					System.out.println(copy.getChildren().get(1).getChildren().get(0));
					substituteArguments(copy.getChildren().get(1).getChildren().get(0).getChildren().get(0));
				}
				
				break;
			default:
				break;
			}	
		return copy; 
	}
	
	
	/**
	 * Get the set of action effects where each parameter has been substituted with the corresponding input.
	 * @return The set of action effects
	 */
	
	public ArrayList<Expression<String>> getEffects(){
		ArrayList <Expression<String>> 	effects = new ArrayList <Expression<String>>();
		//this.effectsNew.removeAll(this.effectsNew);
		for(Expression<String> effect:this.effects ) {
			Expression<String> copy = new Expression<String>(effect);
			switch (effect.getConnector()) {		
				case AT_START:
				case AT_END:
				case OVER_ALL:
					copy= getEffect(effect.getChildren().get(0));				
					break;
				default:
					copy = getEffect(effect);
					break;
				}	
			
			effects.add(copy);
		}
		return effects; 
	}
	
	
	
	/**
	 * Get a copy of the action, where the copy has the same the name, preconditions, effects and parameters of this action.
	 * @return A new copy of this action
	 */
	public Action getCopy() {
		Action copy = new Action();
		copy.name = this.name;
		copy.preconditions = this.preconditions;
		copy.effects = this.effects;
		copy.parameters = this.parameters;
	   return copy;
	}
	

	public String toString() {
		String action = "(" + this.getName() + " ";
		if(!this.inputs.isEmpty()){
			for (String parameter: this.inputs.keySet()) {
				action += this.inputs.get(parameter).getVariable() + " ";
			}
			action = action.trim(); // Remove trailing space
		}
		else{
			for (String parameter: this.parameters) {
				action += parameter + "";
			}
			action = action.trim(); // Remove trailing space
		}
		action += ")";

		return action;
		

		
	}

	
}



