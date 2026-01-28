package org.pofe.athena.tests;

import java.io.*;

import fr.uga.pddl4j.parser.Expression;
import fr.uga.pddl4j.parser.ParsedAction;
import fr.uga.pddl4j.parser.ParsedMethod;
import org.pofe.athena.problems.PlanningProblem;


public class ParserTest {

	public static void main(String[] args) throws Exception {
		// TODO Auto-generated method stub
		
		File domain = new File("src/athena/planning/athena_core/src/main/java/org/pofe/athena/tests/domain.hddl");
		File problem = new File("src/athena/planning/athena_core/src/main/java/org/pofe/athena/tests/pfile01.hddl");
		
		
		//Create a planning problem and set the coordinator
		PlanningProblem planningProblem = new PlanningProblem();
		
		//Parse PDDL problem and domain files
		planningProblem.parse(domain, problem);
		
		//Print all methods and actions
        for(ParsedMethod method : planningProblem.getDomain().getMethods()){
            System.out.println("Method: " + method.getTask());
        }
		for(ParsedAction action : planningProblem.getDomain().getActions()){
			System.out.println("Action: " + action.getName());
			Expression<String> effs = action.getEffects();
			
			for(Expression<String> eff : effs.getChildren()){
				if(eff.getArguments().isEmpty()){			
					System.out.println("Effect: " + eff.getChildren().get(0));
				}
			}
		}
		
		
	}

}
