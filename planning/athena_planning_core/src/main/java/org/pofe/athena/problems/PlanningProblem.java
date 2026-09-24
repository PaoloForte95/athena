package org.pofe.athena.problems;

import java.io.*;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Set;
import java.util.SortedSet;
import java.util.TreeSet;
import java.util.logging.Logger;

import org.jgrapht.graph.DefaultDirectedGraph;
import org.jgrapht.graph.DefaultEdge;

import org.pofe.athena.util.Logging;

//PDDL4j imports
import fr.uga.pddl4j.parser.ParsedAction;
import fr.uga.pddl4j.parser.Connector;
import fr.uga.pddl4j.parser.ParsedDomain;
import fr.uga.pddl4j.parser.ParsedProblem;
import fr.uga.pddl4j.parser.Expression;
import fr.uga.pddl4j.parser.NamedTypedList;
import fr.uga.pddl4j.parser.Parser;
import fr.uga.pddl4j.parser.Symbol;
import fr.uga.pddl4j.parser.TypedSymbol;
//Athena imports
import org.pofe.athena.parser.*;

import org.pofe.athena.plan.*;
import org.pofe.athena.util.State;
import org.pofe.athena.verifier.PlanVerifier;
/**
 * This class provides a combined task and motion planning system for a fleet of robots. If a plan is provided as input to the system, it computes a concurrent 
 * plan by removing unnecessary constraints between actions. Instead, if a plan is not provided, the system evaluates it. 
 * An instantiatable {@link PlanningProblem} required a planning problem and domain files expressed in PDDL. Moreover, each instantiatable 
 * {@link PlanningProblem}  must provide an implementation of the {@link #computePlan()} function 
 * 
 * @author pofe
 *
 */
public class PlanningProblem {

		private static enum TYPE {SYMBOLIC, NUMERICAL, TEMPORAL};
	
		protected Parser parser;
		protected static Logger logger;
		
		//Plan Parameters
		protected AbstractPlan plan;
		
		//Planning Problem parameters
		//protected ArrayList<String> robotsType; //the types of robot defined in the PDDL problem
		//protected ArrayList<String> waypointsType; //the types of waypoint defined in the PDDL problem
		protected List<TypedSymbol<String>> objects;
		protected HashMap <String,  ArrayList <String>> typesDef;
		protected HashMap <String,  ArrayList <String>> typesSet;
		protected HashMap <String, Action> actionsSet;
		protected HashMap <String, Method> methodsSet;
		protected State currentState;
		protected ParsedDomain ParsedDomain;
		protected ParsedProblem ParsedProblem;
		protected HashMap <Integer, Double> robotsCapacities;
		protected HashMap <String, Double> numericVariables;

		protected TYPE problemType;

		protected String planning_problem_file;
		protected String planning_domain_file;

	
		public PlanningProblem() {
			parser = new Parser();
			logger = Logging.getLogger(PlanningProblem.class);
			currentState = new State();
			//robotsType = new ArrayList<String>(); //the types of robot defined in the PDDL problem
			//waypointsType = new ArrayList<String>(); //the types of waypoint defined in the PDDL problem
			objects = new ArrayList <TypedSymbol<String>>();
			typesSet = new HashMap<String, ArrayList <String>>();
			typesDef = new HashMap<String, ArrayList <String>>();
			actionsSet = new HashMap <String, Action>();
			methodsSet = new HashMap <String, Method>();
			robotsCapacities = new HashMap <Integer, Double>();
			numericVariables = new HashMap <String, Double>();
		}


		private void parseMethods(){
		HashMap <String, Method> methods = new HashMap <String, Method>();
		for ( NamedTypedList i : this.getDomain().getTasks()){
			Method method = new Method();
			String name = i.getName().toString();
			method.setName(name);
			methods.putIfAbsent(name, method);
		}
			this.setMethodsSet(methods);
		}


		private void parseActions(){
			
			HashMap <String, Action> actionsSet = new HashMap <String, Action>();
			
			HashMap <String,   ArrayList<String>> typesHasMap = buildTypeHierarchy(this.getDomain().getTypes());
			typesHasMap = parseTypes(this.getProblem().getObjects(), typesHasMap);
			for ( ParsedAction i : this.getDomain().getActions()) {
				Action action = new Action();
				String actionName =  i.getName().toString();
				action.setName(actionName);
				ArrayList <Expression<String>> preconditions = new  ArrayList <Expression<String>>();
				ArrayList <Expression<String>> effects = new  ArrayList <Expression<String>>();
				ArrayList <String> parameters = new  ArrayList <String>();
				int index = this.getDomain().getActions().indexOf(i);
				Expression<String> precondition;
				for (Expression<String> prec : this.getDomain().getActions().get(index).getPreconditions().getChildren()) {
					
					int index2 = this.getDomain().getActions().get(index).getPreconditions().getChildren().indexOf(prec);
					precondition = this.getDomain().getActions().get(index).getPreconditions().getChildren().get(index2);	
					
					if(precondition.getConnector() == Connector.FORALL){
						for (int ob = 0; ob < precondition.getQuantifiedVariables().size(); ob++) {
							TypedSymbol<String> symbol = precondition.getQuantifiedVariables().get(ob);

							//Get all elements specified in the problem with the same type
							for(TypedSymbol<String> init : this.getProblem().getObjects()){
								if(init.getTypes().get(0).toString().equals(symbol.getTypes().get(0).toString())){
									TypedSymbol<String> copy = new TypedSymbol<String>(symbol);

									copy.setValue(init.getValue().toString());
									for(Expression<String> child : precondition.getChildren()){
									Expression<String>  newPrec = new Expression<String>(child.getConnector());
									newPrec.setArguments(precondition.getArguments());
									newPrec.setSymbol(precondition.getSymbol());
									if(child.getConnector() == Connector.NOT){
										for(Expression<String> child2 : child.getChildren()){
											Expression<String> childCopy = new Expression<String>(child2);
											for(Symbol<String> arg : childCopy.getArguments()){
												if(arg.getValue().equals(symbol.getValue().toString())){
													arg.setValue(copy.getValue());
												}
											}	
											newPrec.addChild(childCopy);
										}
									}
									preconditions.add(newPrec);
									}
								}
							}
						}
					continue;
					}
					preconditions.add(precondition);
						
					}
					
					Expression<String> effect;
					for (Expression<String> eff : this.getDomain().getActions().get(index).getEffects().getChildren()) {
						
						int index2 = this.getDomain().getActions().get(index).getEffects().getChildren().indexOf(eff);
						effect = this.getDomain().getActions().get(index).getEffects().getChildren().get(index2);
						effects.add(effect);
						
						
					}
					String parameter ="";
					for (TypedSymbol<String> j : this.getDomain().getActions().get(index).getParameters()) {
						parameter = j.getImage();
						parameters.add(parameter);
						
						
						String type = j.getTypes().get(0).toString();
						String object = j.getImage().toString();
						ArrayList<String> typesSet = new ArrayList<String> ();
						if(!typesHasMap.containsKey(object)) {
							
							typesSet.add(type);
							typesHasMap.put(object, typesSet);
						}
						else {
							typesSet = typesHasMap.get(object);
							if(!typesSet.contains(type)) {
								typesSet.add(type);
								typesHasMap.replace(object, typesSet);
							}
						}
						
					}
					action.addPreconditions(preconditions);
					action.addEffects(effects);
					action.addParameters(parameters);
					actionsSet.put(actionName,action);	
				}
			
			this.setActionsSet(actionsSet);
			this.setTypesSet(typesHasMap);
			this.setObjectsSet(this.getProblem().getObjects());
			this.initializeNumericVariables((this.getProblem().getInit()));
			this.updateState(this.getProblem().getInit());
			}
			
		

		private HashMap<String, ArrayList<String>> buildTypeHierarchy(List<TypedSymbol<String>> typeDefinitions) {
			HashMap<String, ArrayList<String>> typeHierarchy = new HashMap<>();
			
			// First pass: build direct parent relationships
			HashMap<String, String> directParents = new HashMap<>();
			
			for (TypedSymbol<String> typeDef : typeDefinitions) {
				String type = typeDef.getImage().toString().toLowerCase();
				
				if (typeDef.getTypes() != null && !typeDef.getTypes().isEmpty()) {
					// Type has a parent
					String parent = typeDef.getTypes().get(0).toString().toLowerCase();
					directParents.put(type, parent);
				} else {
					// Root type has no parent
					directParents.put(type, null);
				}
			}
			
			// Second pass: build full hierarchy for each type
			for (String type : directParents.keySet()) {
				ArrayList<String> hierarchy = new ArrayList<>();
				String currentType = type;
				
				// Traverse up the hierarchy
				while (currentType != null) {
					hierarchy.add(currentType);
					currentType = directParents.get(currentType);
				}
				
				typeHierarchy.put(type, hierarchy);
			}
			
			return typeHierarchy;
		}	


		
		// private HashMap<String, ArrayList<String>> parseObjectTypes(List<TypedSymbol<String>> objects){
		// 	HashMap <String,  ArrayList<String>> typesHasMap = new HashMap<String,ArrayList<String>>();
		// 		for (TypedSymbol<String> k: objects) {
		// 			String type = k.getTypes().get(0).toString();
		// 			String object = k.getImage().toString();
		// 			ArrayList<String> typesSet = new ArrayList<String> ();
		// 			if(!typesHasMap.containsKey(object)) {
		// 				typesSet.add(type);
		// 				typesHasMap.put(object, typesSet);
		// 			}
		// 			else {
		// 				typesSet = typesHasMap.get(object);
		// 				if(!typesSet.contains(type)) {
		// 					typesSet.add(type);
		// 					typesHasMap.replace(object, typesSet);
		// 				}
		// 			}
		// 		}
		// 		return typesHasMap;
		
		// }


		private HashMap<String, ArrayList<String>> parseTypes(List<TypedSymbol<String>> objects, HashMap<String, ArrayList<String>> typeHierarchy){
			HashMap<String, ArrayList<String>> typesHashMap = new HashMap<>();
			
			for (TypedSymbol<String> k : objects) {
				String object = k.getImage().toString().toLowerCase();
				String directType = k.getTypes().get(0).toString().toLowerCase();
				
				// Get the full hierarchy for this type
				ArrayList<String> fullHierarchy = new ArrayList<>();
				
				if (typeHierarchy.containsKey(directType)) {
					// Copy the full type hierarchy
					fullHierarchy.addAll(typeHierarchy.get(directType));
				} else {
					// If type not in hierarchy, just add the direct type
					fullHierarchy.add(directType);
				}
				
				typesHashMap.put(object, fullHierarchy);
			}
			
			return typesHashMap;
		}
		

		public HashMap<String, Double> initializeNumericVariables(List<Expression<String>> init){
			for(Expression<String> i: init) {
				if(i.getConnector() == Connector.FN_ATOM) {
					System.out.println(i);
					double value = Double.parseDouble(i.getChildren().get(1).toString());
					//Expression<String> expression = i.getChildren().get(0);
					String expression = i.getChildren().get(0).toString();
					numericVariables.put(expression, value);		
				}
				
			}
			if(!numericVariables.isEmpty()) {
				this.problemType = TYPE.NUMERICAL;
			}
			return numericVariables;
		}
		
		
		/**
		 * Parse the domain and the problem files.
		 * @param domain -> the domain file
		 * @param problem -> the problem file
		 * @throws FileNotFoundException
		 */
		public void parse(File domain, File problem) throws FileNotFoundException{
			planning_domain_file = domain.getAbsolutePath();
			planning_problem_file = problem.getAbsolutePath();
			System.out.println("Parsing domain file: " + planning_domain_file);
			System.out.println("Parsing problem file: " + planning_problem_file);
			this.problemType = TYPE.SYMBOLIC;
			this.ParsedDomain = parser.parseDomain(domain);
			this.ParsedProblem = parser.parseProblem(problem);
			if(this.ParsedDomain == null){
				throw new IllegalArgumentException("The domain file is not valid");
			}
			else if (this.ParsedProblem == null){
				throw new IllegalArgumentException("The problem file is not valid");
			}
			parseActions();
			parseMethods();
			
		}

		public String getPlanningProblemFile() {
			return this.planning_problem_file;
		}

		public String getPlanningDomainFile() {
			return this.planning_domain_file;
		}
		
		public ParsedDomain getDomain() {
			return this.ParsedDomain;
		}
		
		public ParsedProblem getProblem() {
			return this.ParsedProblem;
		}
	
		public ArrayList<String> getTypeDef(String type) {
			return this.typesDef.get(type);
		}

		public String getType(){
			return this.problemType.toString();
		}
		
		
		/**
		 * Get an action parameters by using a specific type
		 * @param type -> the type of the parameter to get
		 * @param params -> list of action parameters 
		 * @return parameter -> the action parameter of the indicated type
		 */
		protected String getParameterByType(ArrayList<String> type, ArrayList<String> params) {
			for(String param : params) {
				if( !Collections.disjoint(getType(param),type)) {
					return param;
				}
			}
			return new String();

		}
		
		public State getCurrentState() {
			return this.currentState;
		}
		
		
		public String printCurrentState() {
			return this.currentState.toString();
		}
		
		
		public void updateState(List<Expression<String>> state) {
			
			this.currentState.updateState(state);
		}
		
		public void setActionsSet(HashMap <String, Action> actionsSet){
			this.actionsSet = actionsSet;
		}

		public void setMethodsSet(HashMap <String, Method> methods){
			this.methodsSet = methods;
		}
		
		public void setTypesSet(HashMap <String,  ArrayList <String>> typesSet){
			this.typesSet = typesSet;
		}
		
		public void setObjectsSet(List<TypedSymbol<String>> list) {
			this.objects = list;
		}
			
		
		public HashMap<String, Action> getActions(){
			return this.actionsSet;
		}
		
		public Action getAction(String actionName){
			if(this.actionsSet.containsKey(actionName)) {
				return this.actionsSet.get(actionName).getCopy();
			}
			return null;
		}
		

		public ArrayList <String> getTypes(String param){
			
			if(this.typesSet.containsKey(param)) {
				return this.typesSet.get(param);
			}
			return new ArrayList <String>();
		}

		public ArrayList<String> getType(String param){
			if(this.typesSet.containsKey(param)) {
				return this.typesSet.get(param); 
			}
			return new ArrayList<String>();
		}
		
		public ArrayList<String> getType(Symbol<String> inputVariable){
			return getType(inputVariable.toString());
		}
	
		
		/*
		public String getType(String param){
			if(this.typesSet.containsKey(param)) {
				ArrayList<String> typSet = this.typesSet.get(param); 
				String types = "";
				for (String type : typSet) {
					types += type;
				}
				return types;
			}
			return new String();
		}
		
		
		public String getType(SymbolicSymbol inputVariable){
			return getType(inputVariable.toString());
		}
		*/
		
	
		public Set<String> getActionsNames(){
			return this.actionsSet.keySet();
		}

		/**
		 * Read the plan file and create a graph associated to it
		 * @param plan : a txt file that contains an execution plan computed in PDDL
		 * @return A direct oriented graph representation of the execution plan. The plan is considered as a Total Ordered Plan.
		 */
		public void readPlan (File plan)  { 
			//planningLogger.info("Initial state: " + getCurrentState().toString());
			//Start to read the file 
			BufferedReader br = null;
			try {
				br = new BufferedReader(new FileReader(plan));
			}
			catch (FileNotFoundException e) {
					e.printStackTrace();
			}
			
			if(plan == null) {
				throw new IllegalArgumentException("The input plan is null");

			}
			PlanVerifier planVerifier = new PlanVerifier (this.ParsedProblem.getInit());
			this.plan = new ConcurrentPlan(planVerifier);
			String line; //The current line of the file 
			String[] splittedLine = new String[0];
			int step = 0;
			int ID = 1;
			Set<String> actionsNames = getActionsNames();
			List<Method> methods = new ArrayList<>();
			try {
				while ((line = br.readLine()) != null) {
					line = line.toLowerCase(); // to avoid problems if the plan has some undesired upper cases
					Action action = new Action();
					splittedLine = line.split(" ");
					ArrayList<String> params = new ArrayList<String>();
					for(int i=0; i < splittedLine.length; i++) {
						splittedLine[i] = splittedLine[i].replaceAll("[()]", "");

						if(splittedLine[i].contains("root")){
							methods.addAll(getMethods(splittedLine, br ));
							break;
						}

						//Found an action indicated in the domain file
						if(actionsNames.contains(splittedLine[i])) {
							action = getAction(splittedLine[i]);
							params.addAll(action.getParameters()); 
							//Get the ID
							String IDString = splittedLine[0].replaceAll("[^0-9]", "");
							ID = Integer.parseInt(IDString);
							action.setID(ID);
						} 
						//Associate to each action parameter the corresponding variable indicated in the plan action				 
						ArrayList<String> variableType = getType(splittedLine[i]);	
						SymbolicSymbol input = new SymbolicSymbol(splittedLine[i], variableType);
					
						if(!variableType.isEmpty() && ! getParameterByType(variableType, params).isEmpty()) {
							String param = getParameterByType(variableType, params);	
							action.addInput(param, input);
							params.remove(param);
						}			  			  
					}

					if(!action.getName().isEmpty()) {
						this.plan.add(step, action);
						step +=1;	
					}
					
				}
			} catch (NumberFormatException e) {
						e.printStackTrace();
				} 
				catch (IOException e) {
						e.printStackTrace();
				} 
		for(Method m: methods){
			this.plan.add(m);
		}
		this.plan.createGraph();
		logger.info("Plan length: " + this.plan.makespan());
		}



		private List<Method> getMethods(String[] currentLine, BufferedReader br){
			SortedSet<Integer> tasks = new TreeSet<>();
			List<Method> methods = new ArrayList<>();
			Method method = null;
			String line;

			for(int i=0; i < currentLine.length; i++){
				if(!currentLine[i].matches(".*\\D.*")){
					tasks.add( Integer.parseInt(currentLine[i]));
				}
			}
			try {
				while ((line = br.readLine()) != null) {
					String[] splittedLine = line.split(" ");
					String name = null;
					int ID = -1;
					for(int i=0; i < splittedLine.length; i++){
						String s = splittedLine[i].replaceAll(":", "");
						s = s.replaceAll("[()]", "");
						if(methodsSet.keySet().contains(s) && ID != -1){
							name = s;
							int index = methods.indexOf(method);
							methods.get(index).setName(name);
						}
						//Get ID
						if (s.matches("\\d+")) {
							if(tasks.contains(Integer.parseInt(s))){
								ID = Integer.parseInt(s);
								method = new Method(ID);
								methods.add(method);
								continue;
							}
							for(List<Action> as :getPlan().getActions().values() ){
								for(Action a : as){
									if(a.getID() == Integer.parseInt(s)){
										int index = methods.indexOf(method);
										methods.get(index).addSubtask(Integer.parseInt(s));
									}
								}
							}
						}
					}
				}
			} catch (IOException e) {
				e.printStackTrace();
			}

			return methods;	
		}


		
		public AbstractPlan getPlan() {
			return this.plan;
		}
		
		public DefaultDirectedGraph<Action, DefaultEdge> getGraphPlan() {
			return this.plan.getGraph();
		}

		protected static boolean isNumeric(String str) { 
			  try {  
			    Double.parseDouble(str);  
			    return true;
			  } catch(NumberFormatException e){  
			    return false;  
			  }  
		}

		/**
		 * Update the continuous variables (i.e. numeric fluents) of the planning problem using the set of effects of an action.
		 * @param effects -> the set of effects
		 */
		
		protected void updateNumericVariables(ArrayList<Expression<String>> effects){
			for (Expression<String> effect : effects) {
				 double value = Double.NaN;
				 double currentValue = 0;
				 //If the effect is a numerical effect
				 if(effect.getConnector() == Connector.INCREASE || effect.getConnector() == Connector.DECREASE) {
					 List<Expression<String>> children = effect.getChildren();
					 String valueVariable = children.get(1).toString();
					 Expression<String> variable = children.get(0);
					 //If the increasing/decreasing value is a number
					 if(isNumeric(valueVariable)){
						 currentValue = getValue(variable.toString());
	
						 value = Double.parseDouble(valueVariable);
						 if(effect.getConnector() == Connector.INCREASE) {
							 value =  currentValue + value;
						 }
						 else if(effect.getConnector() == Connector.DECREASE) {
							 value =  currentValue - value;
						 }
						 variable.setValue(value);

						 
					 }
					 //If the increasing/decreasing value is a numerical predicate (e.g. robot_capacity)
					 else if (hasNumericVariable(valueVariable)) {
						 currentValue = getValue(variable.toString());
						 value = getValue(valueVariable);
						 if(effect.getConnector() == Connector.INCREASE) {
							 value =  currentValue + value;
						 }
						 else if(effect.getConnector() == Connector.DECREASE) {
							 value =  currentValue - value;
						 }
						 variable.setValue(value);
					 }
					 //If the increasing/decreasing value is given by an expression
					 else {
						 List<Expression<String>> childrenExpression = effect.getChildren().get(1).getChildren().get(0).getChildren();
						 currentValue = getValue(variable.toString());
						 if(childrenExpression.size() > 0) {
							 value = getValue(childrenExpression.get(0).toString())/getValue(childrenExpression.get(1).toString()); 
						 }
						 else {
							 value = getValue(childrenExpression.get(0).toString());
						 }

						 if(effect.getConnector() == Connector.INCREASE) {
							 value =  currentValue + value;
						 }
						 else if(effect.getConnector() == Connector.DECREASE) {
							 value =  currentValue - value;
						 }
						 variable.setValue(value);
					 }
					 updateNumericVariable(variable.toString(), value);
				 }
			 }
		}

		public double updateNumericVariable(String variable, double value) {
			if(hasNumericVariable(variable)) {
				this.numericVariables.replace(variable, value);
				return this.numericVariables.get(variable);
			}else {
				this.numericVariables.put(variable, value);
			}
			return Double.NaN;
		}
		
		public boolean hasNumericVariable(String variable) {
			return this.numericVariables.containsKey(variable);
		}
		/**
		 * Get the value associated to a numeric predicate of the planning problem
		 * @param variable -> a numeric predicate
		 * @return the actual value of the numeric predicate
		 */
		public double getValue(String variable) {
			if(this.hasNumericVariable(variable)) {
				return this.numericVariables.get(variable);
			}
			return 0;
		}
		
		
		public double getCapacity (int robotID) {
			if(robotsCapacities.containsKey(robotID)) {
				return robotsCapacities.get(robotID);
			}
			return 0.0;
		}

		protected double round (double value, int precision) {
		    int scale = (int) Math.pow(10, precision);
		    return (double) Math.round(value * scale) / scale;
		}
	
		

}