#### SOM Models
###SOM_00:
#Every point in SOM_00 training set was made with a window of 30 points concatenated
#Then, the window moved forward 5 points
#It was used both the acelerometers results (after kalman filter) and the gyroscope.

###Markers_00:
#It has 5 values: position i, position j,list of classes that marked that neuron(0,1,2)
#The size of one row will increase based on number of classes. In the future it will probably have a hot-encoding notation (1 in the column shows that class was marked on the neuron, so multiple classes can mark the same neuron).
