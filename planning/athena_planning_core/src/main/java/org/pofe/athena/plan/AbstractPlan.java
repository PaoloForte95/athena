package org.pofe.athena.plan;

import static org.junit.Assert.assertTrue;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.logging.Logger;

import javax.imageio.ImageIO;
import javax.swing.JFrame;
import javax.swing.JScrollPane;
import javax.swing.SwingConstants;


import org.jgrapht.ext.JGraphXAdapter;
import org.jgrapht.graph.DefaultDirectedGraph;
import org.jgrapht.graph.DefaultEdge;


import com.mxgraph.layout.hierarchical.mxHierarchicalLayout;
import com.mxgraph.swing.mxGraphComponent;
import com.mxgraph.util.mxCellRenderer;

import fr.uga.pddl4j.parser.Connector;
import fr.uga.pddl4j.parser.Expression;

import org.pofe.athena.parser.Action;
import org.pofe.athena.parser.Method;

import org.pofe.athena.util.ExpressionUtil;


public abstract class AbstractPlan implements Plan {
	
	protected DefaultDirectedGraph<Action, DefaultEdge> graphPlan; //Graph representation of the plan
	protected double cost;

	protected List<Method> methods;
	protected static Logger logger;
	

	public AbstractPlan(){
		methods = new ArrayList<>();
	}


	/**
	 * Create the graph using a Plan
	 * @param plan
	 * @return
	 */
	public abstract DefaultDirectedGraph<Action, DefaultEdge>  createGraph();
	
	public DefaultDirectedGraph<Action, DefaultEdge> getGraph(){
		return this.graphPlan;
	}


	public void add(Method m){
		this.methods.add(m);
	}


	public List<Method> getMethods(){
		return this.methods;
	}
	
    @Override
    public void showGraph() {
		
		DefaultDirectedGraph<String, DefaultEdge> graphString = new DefaultDirectedGraph<String, DefaultEdge>(DefaultEdge.class);
  
  	    for(Action action: this.graphPlan.vertexSet()) {
  	    	String name =  "Action #" + Integer.toString(action.getID()) + " " + action.getName() + " : " + action.getInputs();
  	    	graphString.addVertex(name);
  	    }
		JGraphXAdapter<String, DefaultEdge> graphAdapter = new JGraphXAdapter<String, DefaultEdge>(graphString);
		mxHierarchicalLayout layout = new mxHierarchicalLayout(graphAdapter,SwingConstants.NORTH);
		
		layout.execute(graphAdapter.getDefaultParent());

		for(Action act : this.graphPlan.vertexSet()) {
			Set<DefaultEdge> edges = this.graphPlan.edgesOf(act);
			for(DefaultEdge edge: edges) {
				String actionSource = "Action #" + Integer.toString(act.getID()) + " " + act.getName() + " : " + act.getInputs();
				Action tgt = this.graphPlan.getEdgeTarget(edge);
				String actionTarget = "Action #" + Integer.toString(tgt.getID()) + " " + tgt.getName() + " : " + tgt.getInputs();
				if(!graphString.containsEdge(actionSource, actionTarget) && !actionTarget.contains(actionSource)) {
					graphString.addEdge(actionSource, actionTarget);
					graphAdapter.insertEdge(graphAdapter.getDefaultParent(), null, "", 
							graphAdapter.getVertexToCellMap().get(actionSource), 
							graphAdapter.getVertexToCellMap().get(actionTarget));
				
				}
				
			}

		}
		
		JFrame frame = new JFrame("Plan Graph");

		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame.setSize(1000, 1000);
		frame.setResizable(false);

		// Create the mxGraphComponent
		mxGraphComponent graphComponent = new mxGraphComponent(graphAdapter);
		
		// Set the size of the graph component
		graphComponent.setPreferredSize(new Dimension(1000, 1000));

		// Create a JScrollPane and set its view to the mxGraphComponent
		JScrollPane scrollPane = new JScrollPane(graphComponent);


		// Set the scroll pane as the content pane of the frame
		frame.setContentPane(scrollPane);		

		frame.setLocationRelativeTo(null);
		frame.setVisible(true);

  		
    	File imgFile = new File("Plan.png");	
	    try {
			imgFile.createNewFile();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		layout.execute(graphAdapter.getDefaultParent());
		BufferedImage image = mxCellRenderer.createBufferedImage(graphAdapter, null, 2, Color.WHITE, true, null);		
		try {
			ImageIO.write(image, "PNG", imgFile);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}		
		assertTrue(imgFile.exists());
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
		Expression<String> negatedPredicate = ExpressionUtil.negate(predicate);
		if(isContained(negatedPredicate,previousEffects)) {
			isNegated = true;
			
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
		Expression<String> producedPredicate = ExpressionUtil.produce(predicate);
		if(isContained(producedPredicate,previousEffects)) {
			isProduced = true;
		}
		return isProduced;
		
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
	
	
	


    /*public double makespan(DefaultDirectedGraph<Mission, DefaultEdge> Graph) {
	DefaultDirectedGraph<Mission, DefaultEdge> GraphClone = (DefaultDirectedGraph<Mission, DefaultEdge>) Graph.clone();
	TransitiveReduction.INSTANCE.reduce(GraphClone);
	int depth = 0;
	BreadthFirstIterator<Mission, DefaultEdge> it= new BreadthFirstIterator<Mission, DefaultEdge>(GraphClone);
	TopologicalOrderIterator<Mission, DefaultEdge> it2= new TopologicalOrderIterator<Mission, DefaultEdge>(GraphClone); 

	 while(it.hasNext()) {
		  it.next();	  
	  }
	
	while(it2.hasNext()) {
		  Mission m1De = it2.next();
		  if(it.getDepth(m1De) > depth) {
			  depth = it.getDepth(m1De);
		  }
	  }
	makespan = depth;
	return makespan;
	}
	*/

}