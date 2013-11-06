//package ca.nengo.model;

import ca.nengo.model.Node;
import ca.nengo.model.Origin;
import ca.nengo.model.SimulationException;
import ca.nengo.model.SimulationMode;
import ca.nengo.model.StructuralException;
import ca.nengo.model.Termination;
import ca.nengo.model.impl.RealOutputImpl;
import ca.nengo.model.impl.SpikeOutputImpl;
//import ca.nengo.util.VisiblyMutable;
//import ca.nengo.util.VisiblyMutableUtils;
import ca.nengo.model.nef.impl.DecodedOrigin;
import ca.nengo.model.nef.impl.DecodedTermination;

import ca.nengo.dynamics.impl.EulerIntegrator;
import ca.nengo.dynamics.impl.SimpleLTISystem;

import ca.nengo.model.nef.impl.NEFEnsembleImpl;

//import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math.linear.ArrayRealVector;

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;
//import java.lang.CloneNotSupportedException;
//import ca.nengo.model.impl.AbstractNode;

// Used to interface with the python implementation of a Termination
interface NodeTermination extends Termination {
    public float[] get();
}

//public class Learn extends ca.nengo.model.impl.AbstractNode {
public class Learn implements ca.nengo.model.Node {
//public class Learn {
    
    private int counter = 0;
    //private double delta = 0;
    private float rho = 0.2f;

    //TODO: fix this up to be specific for the case it is used for, to be faster
    private String myName;
    private SimulationMode myMode;
    private final Map<String, Origin> myOrigins;
    private final Map<String, Termination> myTerminations;
    private String myDocumentation;


    private DecodedOrigin origin;
    //private Origin origin;
    private NEFEnsembleImpl Y;
    private NEFEnsembleImpl s;
    //private DecodedTermination Y;
    //private DecodedTermination s;
    //private NodeTermination Y;
    //private NodeTermination s;
    //private Termination Y;
    //private Termination s;
    /*
    private ArrayRealVector valY;
    private ArrayRealVector decoder;
    private ArrayRealVector da;
    private ArrayRealVector delta;
    */
    private boolean[] val_Y;
    //private float[] val_Y;
    private float[] val_s;
    private float[][] decoder;
    private float[] da;
    private float delta;

    //private transient List<VisiblyMutable.Listener> myListeners;

    public Learn( String name, DecodedOrigin the_origin, NEFEnsembleImpl s_ens, NEFEnsembleImpl Y_ens ) {
        //super( name );
        super( );
	myName = name;
	myMode = SimulationMode.DEFAULT;
	myOrigins = new LinkedHashMap<String, Origin>(10);
	myOrigins.put(the_origin.getName(), the_origin);
	myTerminations = new LinkedHashMap<String, Termination>(10);
	s = s_ens;
	Y = Y_ens;
	origin = the_origin;
	//origin = origin;
	/*
	float[][] s_transform = new float[1][1];
	float[][] Y_transform = new float[300][300];
	s_transform[0][0] = 1f;
	for ( int i = 0; i < 300; i++ ) {
	    for( int j = 0; j < 300; j++ ) {
		Y_transform[i][j] = 1f;
	    }
	}
	SimpleLTISystem s_dynamics = new SimpleLTISystem(1,1,1);
	SimpleLTISystem Y_dynamics = new SimpleLTISystem(1,1,1);
	EulerIntegrator integrator = new EulerIntegrator();
	try {
	    //s = new DecodedTermination( clone(), "s", transform, dynamics, integrator );
	    //Y = new DecodedTermination( clone(), "Y", transform, dynamics, integrator );
	    s = new DecodedTermination( this, "s", s_transform, s_dynamics, integrator );
	    Y = new DecodedTermination( this, "Y", Y_transform, Y_dynamics, integrator );
	} catch ( StructuralException e ) {
	    System.err.println("Caught StructuralException: " + e.getMessage());
	//} catch ( CloneNotSupportedException e ) {
	}
	*/
    }
    /*
    public void addLearningTerminations( Termination s_term, Termination Y_term ) {
    //public void addLearningTerminations( NodeTermination s_term, NodeTermination Y_term ) {
        s = s_term.get();
	Y = (NodeTermination)Y_term;
        System.out.println("adding them terminators");
	
	//myTerminations.put(s.getName(), s);
	//myTerminations.put(Y.getName(), Y);
    }
    */
    @Override
    public void run( float startTime, float endTime )
    {
        counter += 1;
	if ( counter % 10 == 0 ) {
	    /*
	    delta = -rho * np.array(self.s.get())*0.00001;
	    Y = np.array(list(self.Y.get()))
	    Y.shape = 300,1
	    da = np.dot(Y, delta)
	    decoder = np.array(self.origin.decoders)
	    self.origin.decoders = decoder + da
	    */
            
            
	    try {
		/*
	        System.out.println(Y.getOrigins().length);
	        System.out.println(s.getOrigins().length);
	        System.out.println(Y.getOrigins()[0].getName());
	        System.out.println(Y.getOrigins()[1].getName());
	        System.out.println(Y.getOrigins()[2].getName());
	        System.out.println(Y.getOrigins()[3].getName());
	        System.out.println(s.getOrigins()[0].getName());
	        System.out.println(s.getOrigins()[1].getName());
	        System.out.println(s.getOrigins()[2].getName());
		*/
		//val_s = s.getTermination("state").getOutput(); // (1,)
		//val_s = ((RealOutputImpl)(s.getTermination("state")).getInput()).getValues(); // (1,)
		////val_s = ((RealOutputImpl)(s.getTermination("state")).getInput()).getValues(); // (1,)
		val_s = ((RealOutputImpl)(s.getOrigin("X")).getValues()).getValues();  // (1,)
		//val_s = ((RealOutput)(s.getTermination("S").getInput())).getValues(); // (1,)
		//val_s = ((DecodedTermination)(s.getTermination("S"))).getInput().getValues(); // (1,)
		delta = val_s[0] * -rho * 0.00001f;
		//val_Y = Y.getTermination("Y").getOutput();  // (300,)
		//val_Y = ((RealOutputImpl)(Y.getTermination("state")).getInput()).getValues();  // (300,)
		////val_Y = ((RealOutputImpl)(Y.getTermination("x")).getInput()).getValues();  // (300,)
		val_Y = ((SpikeOutputImpl)(Y.getOrigin("AXON")).getValues()).getValues();  // (300,)
		//val_Y = ((RealOutput)(Y.getTermination("Y").getInput())).getValues();  // (300,)
		//val_Y = ((DecodedTermination)(Y.getTermination("Y"))).getInput().getValues();  // (300,)
		decoder = origin.getDecoders(); // (300, 300)
		//decoder = ((DecodedOrigin)origin).getDecoders(); // (300, 300)
		//float t = 0f;
		int length_i = val_Y.length;
		//int length_j = decoder.length;
		int length_j = decoder[0].length;
		//System.out.println(length_i);
		//System.out.println(length_j);
		//System.out.println(val_s.length);
		//System.out.println(decoder.length);
		//System.out.println(decoder[0].length);
		for ( int i = 0; i < length_i; i++ ) {
		    for ( int j = 0; j < length_j; j++ ) {
			// This is for the case where s is 1D
			//decoder[j][i] = decoder[j][i] + delta * val_Y[i];
			if ( val_Y[i] ) {
			    decoder[i][j] = decoder[i][j] + delta;
			}
			//System.out.println(val_s[0]);
			//t = delta[0] * valY[i] * -rho * 0.00001f;
		    }
		}
		origin.setDecoders( decoder );
		//((DecodedOrigin)origin).setDecoders( decoder );
	    } catch (StructuralException e) {
		System.err.println("Caught StructuralException: " + e.getMessage());
	    } catch (SimulationException e) {
		System.err.println("Caught SimulationlException: " + e.getMessage());
	    } catch (NullPointerException e) {
		System.err.println("Caught NullPointerException: " + e.getMessage());
		try{
	        //System.out.println(s.getTerminations()[0].getName());
	        System.out.println(Y.getTerminations().length);
	        System.out.println(s.getTerminations().length);
	        System.out.println(Y.getTerminations()[2].getName());
	        System.out.println(s.getTerminations()[1].getName());
	        System.out.println(val_Y);
	        System.out.println(val_s);
	        //System.out.println(val_s.length);
	        System.out.println(((RealOutputImpl)(Y.getTermination("state").getInput())).getValues());
	        System.out.println(((RealOutputImpl)(s.getTermination("state").getInput())).getValues());
		System.out.println(delta);
		System.out.println(origin);
		} catch(StructuralException f) {
		}
		System.out.println(Y);
	    }
	    
	    /*
	    val_s = ((DecodedTermination)s).getOutput(); // (1,)
	    delta = val_s[0] * -rho * 0.00001f;
	    val_Y = ((DecodedTermination)Y).getOutput();  // (300,)
	    decoder = ((DecodedOrigin)origin).getDecoders(); // (300, 300)
	    //float t = 0f;
	    int length = val_Y.length;
	    for ( int i = 0; i < length; i++ ) {
		for ( int j = 0; j < length; j++ ) {
		    // This is for the case where s is 1D
		    decoder[i][j] = decoder[i][j] + delta * val_Y[i];
		    //t = delta[0] * valY[i] * -rho * 0.00001f;
		}
	    }
	    ((DecodedOrigin)origin).setDecoders( decoder );
            */
            

	    /*
	    try {
		
		//delta = -rho * ArrayRealVector( myTerminationsget("s").getOutput() ) * 0.00001;
		/
		delta = ArrayRealVector( myTerminations.get("s").getOutput() 
				        ).mapMultiplyToSelf( -rho * 0.00001);
		valY = ArrayRealVector( myTerminations.get("Y").getOutput() );
		da = valY.dotProduct( delta );
		decoder = ArrayRealVector( getOrigin( "origin" ).getDecoders() );
	        getOrigin( "origin" ).setDecoders( ( decoder.add( da ) ).toArray() );
		/
		
		/
		delta = ArrayRealVector( s.getOutput() ).mapMultiplyToSelf( -rho * 0.00001);
		valY = ArrayRealVector( Y.getOutput() );
		da = valY.dotProduct( delta );
		decoder = ArrayRealVector( origin.getDecoders() );
	        origin.setDecoders( ( decoder.add( da ) ).toArray() );
		/
                
		delta = s.getOutput();
		valY = Y.getOutput();
                float t = 0f;
		int length = delta.length;
		for ( int i = 0; i < length; i++ ) {
		    t = delta[i] * valY[i] * -rho * 0.00001f;
		}

	    } catch (StructuralException e) {
	    }
	    */
	}
    }
    
    //TEMP: testing
    public Termination get_s() {
	return s.getTerminations()[0];
    }
    public Termination get_Y() {
	return Y.getTerminations()[0];
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
	return "";
    }
    
    @Override
    public void setName( String name ) {
	return;
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
