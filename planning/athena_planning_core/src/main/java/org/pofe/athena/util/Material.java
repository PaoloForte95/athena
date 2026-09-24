package org.pofe.athena.util;


import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryFactory;

import org.pofe.athena.util.materialsTypes.MATERIAL_TYPE;;

/**
 * The {@link Material} data structure represents a material in the planning problem: It consists of a material type, a position and an occupancy area fields. 
 * 
 * @author pofe
 *
 */
public class Material {
	
	protected int materialID;
	protected MATERIAL_TYPE materialType ;
	protected double amount = 0;
	protected double height = 1;
	protected Geometry occupancyArea ;
	
	
	//Default occupancy map of the material is set = 0
	protected static Coordinate[] DEFAULT_occupancy_Area = new Coordinate[] {
		  	new Coordinate(0.0, 0.0),	
			new Coordinate(0.0, 0.0),	
			new Coordinate(0.0, 0.0),	
			new Coordinate(0.0, 0.0)	
	};
	
	
	/**
 	 * Create a material instance of unknown type, with the default occupancy area (i.e. =0).
	 *  @param materialID -> the ID of material
	 *  
	 */
	public Material (int materialID) {
		this(materialID, MATERIAL_TYPE.UNKNOWN, DEFAULT_occupancy_Area);
		
	}
	

	/**
 	 * Create a material instance of a defined type, with the default occupancy area (i.e. =0).
	 *  @param materialID -> the ID of material
	 *  @param materialType -> the material type (see {@link MATERIAL_TYPE})
	 *  
	 */
	public Material (int materialID, MATERIAL_TYPE materialType) {
		this(materialID, materialType, DEFAULT_occupancy_Area);
		
	}
	
	/**
 	 * Create a material instance of unknown type, with the defined occupancy area
	 *  @param materialID -> the ID of material
	 *  @param coordinates -> coordinates representing the occupancy map of the material (at least 2 points are required)
	 */
	public Material (int materialID, Coordinate ... coordinates ) {
		this(materialID, MATERIAL_TYPE.UNKNOWN, coordinates);
	}
	
	
	/**
 	 * Create a material instance of a defined type, with the defined occupancy area.
	 *  @param materialID -> the ID of material
	 *  @param materialType -> the material type (see {@link MATERIAL_TYPE})
	 *  @param coordinates -> coordinates representing the occupancy map of the material (at least 2 points are required)
	 */
	public Material (int materialID, MATERIAL_TYPE materialType, Coordinate ... coordinates ) {
		this.materialID = materialID;
		this.materialType = materialType;
		GeometryFactory gf = new GeometryFactory();
		
		 Coordinate[] newCoords = new Coordinate[coordinates.length+1];
			for (int i = 0; i < coordinates.length; i++) {
				newCoords[i] = coordinates[i];
			}
			newCoords[newCoords.length-1] = coordinates[0];
		
		this.occupancyArea = gf.createPolygon(coordinates);
	}
	
	/** 
	 * Get the material ID
	 * @return
	 */
	
	public int getmaterialID() {
		return this.materialID;	
	}
	
	
	/** 
	 * If the material is stacked, set the height (i.e. the level) of the material on the pile. If the material is not stacked (i.e. material is on the ground), 
	 * the height equals 1
	 * @param materialHeight -> the material height 
	 */
	
	public void setHeight(double materialHeight) {
		this.height = materialHeight;
		
			
	}
	
	/** 
	 * Get the "height" of the material on the pile. If the material is not stacked (i.e. material is on the ground), 
	 * the height equals 1
	 * @return
	 */
	public double getHeight() {
		return this.height;
		
			
	}
	
	
	/** 
	 * Check if the material is stacked 
	 * @return -> <code>true</code> if material is stacked, otherwise <code>false</code>
	 */
	public boolean isStacked() {
		return(this.height != 1);
	}
	
	public double getAmount() {
		return this.amount;
	}
	
	public void setAmount(double amountMaterial) {
		this.amount = amountMaterial;
	}
	
	/**
	 * Return the area occupied by the material in the map. This area is considered as an obstacle in the map.
	 * @return occupancyArea -> the material occupancy area
	 */
	public Geometry getOccupancyArea() {
		return this.occupancyArea;
	}
	
	
	
	
	
}
