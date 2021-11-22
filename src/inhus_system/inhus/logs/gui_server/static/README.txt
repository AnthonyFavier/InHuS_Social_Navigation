# How to generate the data file for each map #

The data file is composed of 4 fields and parsed with spaces :

- "image:" : name of the image file for the map (usually same as pgm used in move base but twice bigger and in png)
- "size:" : size in pixel of the image file
- "resolution:" : scaling of the image meter per pixel 
- "offset:" : offset of the origin of the map in meter
- "angle" : angle to rotate the map in 'rad'