**Program**: Flag motion Simulation using mass-spring systems

**Author**: Anirudh Ravi

**About**:

A flag is a sheet of masses connected together by springs. The masses will move around in space due to forces acting on them - e.g. wind, gravity, or springs between particles. 

Created a globe that interacts with the cloth. This interaction showcases the following: gravity simulation, wind effects and elastic properties. Techniques such as Lighting Effects, and Textures have been used to enhance the object’s look and the scene.

**References**:

The following links were very helpful while creating the cloth:

1. Cloths using spring mass systems: http://freespace.virgin.net/hugo.elias/models/m_cloth.htm
2. Paul’s Cloth simulation: http://www.paulsprojects.net/opengl/cloth/cloth.html
3. Texturing tutorial: http://www.codeincodeblock.com/2012/05/simple-method-for-texture-mapping-on.html

**Executing**:

g++ IndianFlag.cpp imageloader.cpp -lGL -lGLU -lglut -lm -o IndianFlag

./IndianFlag
