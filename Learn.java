//package ca.nengo.model;

import ca.nengo.model.Node;
import ca.nengo.model.Origin;
import ca.nengo.model.SimulationException;
import ca.nengo.model.SimulationMode;
import ca.nengo.model.StructuralException;
import ca.nengo.model.Termination;
import ca.nengo.model.impl.RealOutputImpl;
import ca.nengo.model.impl.SpikeOutputImpl;
import ca.nengo.model.nef.impl.DecodedOrigin;
import ca.nengo.model.nef.impl.DecodedTermination;

import ca.nengo.dynamics.impl.EulerIntegrator;
import ca.nengo.dynamics.impl.SimpleLTISystem;

import ca.nengo.model.nef.impl.NEFEnsembleImpl;


import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;


public class Learn implements ca.nengo.model.Node {
    
    private int counter = 0;
    private float rho = 0.2f;

    //TODO: fix this up to be specific for the case it is used for, to be faster
    private String myName;
    private SimulationMode myMode;
    private final Map<String, Origin> myOrigins;
    private final Map<String, Termination> myTerminations;
    private String myDocumentation;


    private DecodedOrigin origin;
    private NEFEnsembleImpl Y;
    private NEFEnsembleImpl s;
    private boolean[] val_Y;
    private float[] val_s;
    private float[][] decoder;
    private float[] da;
    private float delta;

    private float decay;
    private float filtered;
    private double dt=0.001;

    public Learn( String name, DecodedOrigin the_origin, NEFEnsembleImpl s_ens, NEFEnsembleImpl Y_ens ) {
        super( );
	myName = name;
	myMode = SimulationMode.DEFAULT;
	myOrigins = new LinkedHashMap<String, Origin>(10);
	myOrigins.put(the_origin.getName(), the_origin);
	myTerminations = new LinkedHashMap<String, Termination>(10);
	s = s_ens;
	Y = Y_ens;
	origin = the_origin;
	decay = (float)Math.exp(-dt/0.01);
	//decay = (float)Math.exp(-dt/0.001);
	//decay = (float)Math.exp(-dt/0.1);
    }
    @Override
    public void run( float startTime, float endTime )
    {
        counter += 1;
	if ( counter % 10 == 0 ) {
            
	    try {
		val_s = ((RealOutputImpl)(s.getOrigin("X")).getValues()).getValues();  // (1,)
		filtered = filtered*decay + val_s[0] * (1-decay);
		delta = filtered * -rho * 0.00001f * 0.1f;
		val_Y = ((SpikeOutputImpl)(Y.getOrigin("AXON")).getValues()).getValues();  // (300,)
		decoder = origin.getDecoders(); // (300, 300)
		int length_i = val_Y.length;
		int length_j = decoder[0].length;
		for ( int i = 0; i < length_i; i++ ) {
		    for ( int j = 0; j < length_j; j++ ) {
			// This is for the case where s is 1D
			if ( val_Y[i] ) {
			    decoder[i][j] = decoder[i][j] + delta;
			}
		    }
		}
		origin.setDecoders( decoder );
	    } catch (StructuralException e) {
		System.err.println("Caught StructuralException: " + e.getMessage());
	    } catch (SimulationException e) {
		System.err.println("Caught SimulationException: " + e.getMessage());
	    } catch (NullPointerException e) {
		System.err.println("Caught NullPointerException: " + e.getMessage());
	    }
	}
    }
    @Override
    public void addChangeListener( Listener listener ) {
	return;
    }
    
    @Override
    public void removeChangeListener( Listener listener ) {
	return;
    }

    @Override
    public void setMode( SimulationMode mode ) {
	return;
    }
    
    @Override
    public SimulationMode getMode() {
	return null;
    }

    @Override
    public void reset( boolean reset ) {
	return;
    }

    @Override
    public String getName() {
	return myName;
    }
    
    @Override
    public void setName( String name ) {
	myName = name;
    }
    
    @Override
    public Origin[] getOrigins() {
	return myOrigins.values().toArray(new Origin[0]);
    }
    
    @Override
    public Origin getOrigin(String name) throws StructuralException {
	return myOrigins.get(name);
    }

    @Override
    public Termination[] getTerminations() {
        return myTerminations.values().toArray(new Termination[0]);
    }

    @Override
    public Termination getTermination(String name) throws StructuralException {
	return myTerminations.get(name);
    }

    @Override
    public Node[] getChildren() {
        return null;
    }

    @Override
    public String toScript( HashMap<String, Object> scriptData) {
	return "";
    }
    @Override
    public String getDocumentation() {
	return "";
    }
    @Override
    public void setDocumentation( String text ) {
	return;
    }
    
    @Override
    public Node clone() throws CloneNotSupportedException {
        Node result = (Node) super.clone();
	return result;
    }

}
