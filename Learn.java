//package ca.nengo.model;

import ca.nengo.model.Node;
import ca.nengo.model.Origin;
import ca.nengo.model.SimulationException;
import ca.nengo.model.SimulationMode;
import ca.nengo.model.StructuralException;
import ca.nengo.model.Termination;
//import ca.nengo.util.VisiblyMutable;
//import ca.nengo.util.VisiblyMutableUtils;
import ca.nengo.model.nef.impl.DecodedOrigin;
import ca.nengo.model.nef.impl.DecodedTermination;

//import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math.linear.ArrayRealVector;

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;
//import java.lang.CloneNotSupportedException;
//import ca.nengo.model.impl.AbstractNode;

//public class Learn extends ca.nengo.model.impl.AbstractNode {
public class Learn implements ca.nengo.model.Node {
//public class Learn {
    
    private int counter = 0;
    //private double delta = 0;
    private float rho = 0.2f;
    //private Origin origin;
    //private Termination s; 
    //private Termination Y; 

    //TODO: fix this up to be specific for the case it is used for, to be faster
    private String myName;
    private SimulationMode myMode;
    private final Map<String, Origin> myOrigins;
    private final Map<String, Termination> myTerminations;
    private String myDocumentation;


    private DecodedOrigin origin;
    private DecodedTermination Y;
    private DecodedTermination s;
    /*
    private ArrayRealVector valY;
    private ArrayRealVector decoder;
    private ArrayRealVector da;
    private ArrayRealVector delta;
    */
    private float[] valY;
    private float[][] decoder;
    private float[] da;
    private float[] delta;

    //private transient List<VisiblyMutable.Listener> myListeners;

    public Learn( String name, Origin origin ) {
        //super( name );
        super( );
	myName = name;
	myMode = SimulationMode.DEFAULT;
	myOrigins = new LinkedHashMap<String, Origin>(10);
	myOrigins.put(origin.getName(), origin);
	myTerminations = new LinkedHashMap<String, Termination>(10);
	//origin = origin;
    }

    public void addLearningTerminations( Termination s, Termination Y ) {
        s = s;
	Y = Y;
	
	//myTerminations.put(s.getName(), s);
	//myTerminations.put(Y.getName(), Y);
    }

    public void tick()
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
	    delta = s.getOutput();
	    valY = Y.getOutput();
	    float t = 0f;
	    int length = delta.length;
	    for ( int i = 0; i < length; i++ ) {
	        t = delta[i] * valY[i] * -rho * 0.00001f;
	    }
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
    public void run( float startTime, float endTime ) {
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
