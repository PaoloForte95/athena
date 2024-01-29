package org.athena.athena_protobuf;

import java.io.*;
import java.util.List;
import java.util.Map;
import java.util.ArrayList;
import java.util.HashMap;

import org.athena.athena_protobuf.Action;
import org.athena.athena_protobuf.ExecutionPlan;
import org.athena.athena_protobuf.Method;

import com.google.ortools.Loader;
import com.google.ortools.sat.CpModel;
import com.google.ortools.sat.CpSolver;
import com.google.ortools.sat.CpSolverSolutionCallback;
import com.google.ortools.sat.Literal;


public class Allocator {

    private double estimateCycles(double amount, double capacity){
        int cycles = (int) (amount/(1 * capacity));
        return cycles;
    }

    public int[][] optimize(List<Action> actions, List<Method> methods){
        Loader.loadNativeLibraries();
        List<Integer> machines = new ArrayList<>();
        for (Action act: actions){
            if(!machines.contains(act.getRobotID())){
                machines.add(act.getRobotID());
            }
       
        }
        System.out.println("List of robots: " + machines);

        CpModel model = new CpModel();

        Literal[][] variables = new Literal[machines.size()][methods.size()];
        for(int machine: machines){
            int i = machines.indexOf(machine);
            for(Method method : methods){
                int j = methods.indexOf(method);
                variables[i][j] = model.newBoolVar("x"+"["+machine+","+method+"]");
            }
        }
        // Each method is assigned to exactly one machine.
        for(Method method : methods){
            int j = methods.indexOf(method);
            List<Literal> cm = new ArrayList<>();
            for(int machine: machines){
                int i = machines.indexOf(machine);
                cm.add(variables[i][j]);
            }
            model.addExactlyOne(cm);
        }
        //Evaluate costs
        HashMap<Integer, Double> capacities = new HashMap<Integer, Double>();
        HashMap<Integer, Double> amounts = new HashMap<Integer, Double>();
        
        for(int machine : machines){
            capacities.put(machine, 10.0);
        }

        amounts.put(1,30.0);
        amounts.put(2,10.0);
        amounts.put(3,20.0);

        HashMap<Integer, Integer> piles = new HashMap<Integer, Integer>();
        for(Method method : methods){
            for(Action act: actions){
                if (method.getActionsIdsList().contains(act.getId())){
                    int matID = act.getMaterial();
                    piles.put(method.getId(), matID);
                    if(matID >0){
                        break;
                    }
                }
            }
        }
                
        //Initialize cost
        double [][] costs = new double [ machines.size()][methods.size()];	
        for (int i = 0; i < costs.length; i++) {
            int ID = machines.get((i));
            for (int j = 0; j < costs[0].length; j++) {
                int taskID = methods.get((j)).getId();
                double amount = amounts.get(piles.get(taskID));
                costs[i][j] = estimateCycles(amount, capacities.get(ID));
            }
        }
        int [][] current = new int [ machines.size()][methods.size()];	
        int [][] optimal = new int [ machines.size()][methods.size()];	
        CpSolver solver = new CpSolver();

        solver.getParameters().setLinearizationLevel(0);
        solver.getParameters().setEnumerateAllSolutions(true);
        class SolutionPrinterWithLimit extends CpSolverSolutionCallback {

            private int solutionCount = 0;
            private double optimal_cost = 1000000;
            private final List<Integer> machines;
            private final List<Method> methods;
            private final Literal[][] variables;
            
            public SolutionPrinterWithLimit(
                List<Integer> machines, List<Method> methods, Literal[][] variables) {
                solutionCount = 0;
                this.machines = machines;
                this.methods = methods;
                this.variables = variables;
                }

            @Override
            public void onSolutionCallback() {
              System.out.printf("Solution #%d:%n", solutionCount);
                double current_cost = 0;
               
                for (int i = 0; i < machines.size(); i++) {
                     double single_cost = 0;
                    int robotID = machines.get((i));
                    for (int j = 0; j < methods.size(); j++) {
                        int taskID = methods.get((j)).getId();
                        current[i][j] = booleanValue(variables[i][j]) ? 1 : 0;  
                        if(current[i][j] == 1){
                            single_cost += costs[i][j];
                            for (int k = 0; k < j; k++) {
                                if(current[i][k] ==1 ){
                                    single_cost += costs[i][k];
                                }
                            }
                            System.out.println("Single cost(" + robotID + ", "  + taskID + "):" + single_cost);
                            
                        }
                    }

                    current_cost = Math.max(current_cost,single_cost);
                }
                for(int i=0; i< current.length;i ++) {
                        int robotID = machines.get((i));
                        for(int j = 0 ; j <current[0].length; j++) {
                            int taskID = methods.get((j)).getId();
                                if (current[i][j] == 1) {
                                    System.out.println("Robot " + robotID +" is assigned to Task "+ taskID );
                                }	
                        }
                 }
                System.out.println("Makespan: " + current_cost);
                if(current_cost <= optimal_cost){
                    optimal_cost = current_cost;
                    for(int i=0; i< optimal.length;i ++) {
                        for(int j = 0 ; j <optimal[0].length; j++) {
                                optimal[i][j] = current[i][j];
                        }
                    }
                }
                
              solutionCount++;
              
            }

        }
        SolutionPrinterWithLimit cb = new SolutionPrinterWithLimit(machines,methods,variables);
        solver.solve(model,cb);
        System.out.println("----------------------------------");
        System.out.println("Optimal Assignment" );
        for (int i = 0; i < optimal.length; i++) {
			int robotID = machines.get((i));
			for (int j = 0; j < optimal[0].length; j++) {
				int taskID = methods.get((j)).getId();
					if (optimal[i][j] == 1) {
						System.out.println("Robot " + robotID +" is assigned to Task "+ taskID );
					}	
			} 
		}
        return  optimal;
    }

    public static void main(String[] args) throws Exception {
        if (args.length != 1) {
            System.err.println("Data file is missing");
            System.exit(-1);
        }

        // Read the existing address book.
        ExecutionPlan plan = ExecutionPlan.parseFrom(new FileInputStream(args[0]));
        ExecutionPlan.Builder optimizedPlan = ExecutionPlan.newBuilder();
        Allocator allocator = new Allocator();
        allocator.optimize(plan.getActionList(), plan.getMethodList());

        /*for (Method method : plan.getMethodList()){
            System.out.println("Method: " + method.getName());
            System.out.println("Actions: " + method.getActionsIdsList());
            optimizedPlan.addMethod(method);
            for(Action action : plan.getActionList()){
                Action.Builder act = Action.newBuilder(action);
                act.setRobotID(1);
                for(int ID : method.getActionsIdsList()){
                    if(ID == action.getId()){
                        System.out.println("Action " + action.getName() + " assigned to robot " + action.getRobotID());
                        optimizedPlan.addAction(act);
                    }
                }
            }
        }
        for (Method method : optimizedPlan.getMethodList()){
            for(Action action : optimizedPlan.getActionList()){
                for(int ID : method.getActionsIdsList()){
                    if(ID == action.getId()){
                        System.out.println("Action Optimal " + action.getName() + " assigned to robot " + action.getRobotID());
                    }
                }
            }
        }
        */
    }
}
