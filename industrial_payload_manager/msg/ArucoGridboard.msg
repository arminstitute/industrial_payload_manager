#Represents an OpenCV Aruco gridboard

# The number of markes in the X and Y direction
int32 markersX
int32 markersY

# The marker length in meters
float64 markerLength

#The spacing between markers in meters
float64 markerSpacing

#The name of the dictionary, for instance DICT_6X6_250
string dictionary

#The first marker id in the gridboard
int32 firstMarker
