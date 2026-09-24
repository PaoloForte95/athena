package org.pofe.athena.plan;


import java.io.Serializable;
import java.util.List;
import java.util.Map;

import org.jgrapht.graph.DefaultDirectedGraph;
import org.jgrapht.graph.DefaultEdge;

import org.pofe.athena.parser.Action;
import org.pofe.athena.parser.Method;


public interface Plan extends Serializable {

	double makespan();
	
	void showGraph();
	
	boolean add(int step, final Action action);
	
	boolean remove(int step, final Action action);
	
	boolean remove(int step);

	boolean contains(int step, final Action action);
	
	boolean isEmpty();
	
	Map<Integer, List<Action>> getActions();

	List<Method> getMethods();

	List<Action> getActions(int step);
    
    double GetCost();
    
    DefaultDirectedGraph<Action, DefaultEdge> getGraph();
    
    


}