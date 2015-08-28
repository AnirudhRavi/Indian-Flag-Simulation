#include <GL/gl.h>
#include <GL/glut.h> 
#include <math.h>
#include <vector>
#include <iostream>
#include "imageloader.h"

#define DAMPING 0.01 // Damping effect on particles of cloth
#define TIME_STEPSIZE2 0.5*0.5 // Step of time taken by each particle per frame
#define SPRING_ITERATIONS 15 // 

//Defines the class used to set position, force and velocity vectors
class Vec3 
{	
public:
	float f[3];

	Vec3(float x, float y, float z)
	{
		f[0] =x;
		f[1] =y;
		f[2] =z;
	}

	Vec3() {}

	float length()
	{
		return sqrt(f[0]*f[0]+f[1]*f[1]+f[2]*f[2]);
	}

	Vec3 normalized()
	{
		float l = length();
		return Vec3(f[0]/l,f[1]/l,f[2]/l);
	}

	void operator+= (const Vec3 &v)
	{
		f[0]+=v.f[0];
		f[1]+=v.f[1];
		f[2]+=v.f[2];
	}

	Vec3 operator/ (const float &a)
	{
		return Vec3(f[0]/a,f[1]/a,f[2]/a);
	}

	Vec3 operator- (const Vec3 &v)
	{
		return Vec3(f[0]-v.f[0],f[1]-v.f[1],f[2]-v.f[2]);
	}

	Vec3 operator+ (const Vec3 &v)
	{
		return Vec3(f[0]+v.f[0],f[1]+v.f[1],f[2]+v.f[2]);
	}

	Vec3 operator* (const float &a)
	{
		return Vec3(f[0]*a,f[1]*a,f[2]*a);
	}

	Vec3 operator-()
	{
		return Vec3(-f[0],-f[1],-f[2]);
	}

	Vec3 cross(const Vec3 &v)
	{
		return Vec3(f[1]*v.f[2] - f[2]*v.f[1], f[2]*v.f[0] - f[0]*v.f[2], f[0]*v.f[1] - f[1]*v.f[0]);
	}

	float dot(const Vec3 &v)
	{
		return f[0]*v.f[0] + f[1]*v.f[1] + f[2]*v.f[2];
	}
};

//Particle of Cloth
class Mass
{
private:
	bool movable; //Used to fix points on the cloth

	float mass;   // Mass of each particle
	Vec3 pos;     // Current position
	Vec3 old_pos; // Previous position
	Vec3 acceleration; // Acceleration due to forces acting
	Vec3 accumulated_normal; // Non-normalized

public:
	Mass(Vec3 po) 
    { 
                  pos = po; old_pos = po; acceleration = Vec3(0,0,0); mass = 1; movable = true; accumulated_normal = Vec3(0,0,0);
    }
    Mass(){}     //Default Constructor

	void addForce(Vec3 f)
	{
		acceleration += f/mass;           //Calculate acc. on each particle
	}

	//Verlet integration
	void timeStep()
	{
		if(movable)
		{
			Vec3 temp = pos;
			pos = pos + (pos-old_pos)*(1.0-DAMPING) + acceleration*TIME_STEPSIZE2;
			old_pos = temp;
			acceleration = Vec3(0,0,0); // Reseting acceleration for each time step
		}
	}

	Vec3& getPos() {return pos;}

	void resetAcceleration() {acceleration = Vec3(0,0,0);}

	void offsetPos(const Vec3 v) { if(movable) pos += v;}

	void makeUnmovable() {movable = false;}

	void addToNormal(Vec3 normal)
	{
		accumulated_normal += normal.normalized();
	}

	Vec3& getNormal() { return accumulated_normal;} // Normal on each particle

	void resetNormal() {accumulated_normal = Vec3(0,0,0);}
};

class Spring
{
private:
	float rest_distance; // rest length of spring

public:
	Mass *p1, *p2; 

	Spring(Mass *pi, Mass *pj) 
	{
		p1 = pi;
		p2 = pj;
        Vec3 vec = p1->getPos() - p2->getPos();
		rest_distance = vec.length();
	}

    //Calculating constraint between 2 particles, and offset
	void satisfySpring()
	{
		Vec3 p1_to_p2 = p2->getPos() - p1->getPos(); 
		float current_distance = p1_to_p2.length(); 
		Vec3 correctionVector = p1_to_p2*(1 - rest_distance/current_distance)*0.5; // The offset vector that moves p1 into a distance of rest_distance to p2
		Vec3 correctionVectorHalf = correctionVector*0.5; 
		p1->offsetPos(correctionVectorHalf); 
		p2->offsetPos(-correctionVectorHalf); 	
	}
};

class Cloth
{
private:

	int num_particles_width; // particles along width
	int num_particles_height; // particles along height
	// total number of particles is num_particles_width*num_particles_height

	std::vector<Mass> particles; // All the particles
	std::vector<Spring> springs; // All the springs
	
	Mass* getParticle(int x, int y) {return &particles[y*num_particles_width + x];}        //Get particle at row y and column x
	void makeSpring(Mass *p1, Mass *p2) {springs.push_back(Spring(p1,p2));}                //Add elements to the spring list


	//Calculate normal for forces acting on a traingle formed by p1, p2 and p3
	Vec3 calcTriangleNormal(Mass *p1,Mass *p2,Mass *p3)
	{
		Vec3 pos1 = p1->getPos();
		Vec3 pos2 = p2->getPos();
		Vec3 pos3 = p3->getPos();

		Vec3 v1 = pos2-pos1;
		Vec3 v2 = pos3-pos1;

		return v1.cross(v2);
	}

	//Calc wind force acting a traiangle of particles
	void addWindForcesForTriangle(Mass *p1,Mass *p2,Mass *p3, const Vec3 direction)
	{
		Vec3 normal = calcTriangleNormal(p1,p2,p3);
		Vec3 d = normal.normalized();
		Vec3 force = normal*(d.dot(direction));
		p1->addForce(force);
		p2->addForce(force);
		p3->addForce(force);
	}

	//Draw the triangle. Called in drawShade() function
	void drawTriangle(Mass *p1, Mass *p2, Mass *p3, const Vec3 color)
	{
		glColor3fv( (GLfloat*) &color );
		
		Vec3 normal1 = p1->getNormal().normalized();
		Vec3 normal2 = p2->getNormal().normalized();
		Vec3 normal3 = p3->getNormal().normalized();

		glNormal3fv((GLfloat *) &(normal1 ));
		glVertex3fv((GLfloat *) &(p1->getPos() ));

		glNormal3fv((GLfloat *) &(normal2 ));
		glVertex3fv((GLfloat *) &(p2->getPos() ));

		glNormal3fv((GLfloat *) &(normal3 ));
		glVertex3fv((GLfloat *) &(p3->getPos() ));
	}

public:

	//Constructor for Cloth
	Cloth(float width, float height, int num_particles_w, int num_particles_h) 
	{
		num_particles_width = num_particles_w; 
        num_particles_height = num_particles_h;
        
        particles.resize(num_particles_width*num_particles_height); //Total particles in Cloth 

		// creating particles in a grid of particles from (0,0,0) to (width,-height,0)
		for(int x=0; x<num_particles_width; x++)
		{
			for(int y=0; y<num_particles_height; y++)
			{
				Vec3 pos = Vec3(width * (x/(float)num_particles_width),
								-height * (y/(float)num_particles_height),
								0);
				particles[y*num_particles_width+x]= Mass(pos); // insert particle in column x at y'th row
			}
		}

		// Connecting immediate neighbor particles with springs (distance 1 )
		for(int x=0; x<num_particles_width; x++)
		{
			for(int y=0; y<num_particles_height; y++)
			{
				if (x<num_particles_width-1) makeSpring(getParticle(x,y),getParticle(x+1,y));
				if (y<num_particles_height-1) makeSpring(getParticle(x,y),getParticle(x,y+1));
				if (x<num_particles_width-1 && y<num_particles_height-1) makeSpring(getParticle(x,y),getParticle(x+1,y+1));
				if (x<num_particles_width-1 && y<num_particles_height-1) makeSpring(getParticle(x+1,y),getParticle(x,y+1));
			}
		}


		// Connecting 2nd neighbors with springs (distance 2 )
		for(int x=0; x<num_particles_width; x++)
		{
			for(int y=0; y<num_particles_height; y++)
			{
				if (x<num_particles_width-2) makeSpring(getParticle(x,y),getParticle(x+2,y));
				if (y<num_particles_height-2) makeSpring(getParticle(x,y),getParticle(x,y+2));
				if (x<num_particles_width-2 && y<num_particles_height-2) makeSpring(getParticle(x,y),getParticle(x+2,y+2));
				if (x<num_particles_width-2 && y<num_particles_height-2) makeSpring(getParticle(x+2,y),getParticle(x,y+2));			}
		}


		// making the upper left most three and bottom most three particles unmovable
		for(int i=0;i<3; i++)
		{
			getParticle(0+i ,0)->offsetPos(Vec3(0.5,0.0,0.0)); // moving the particle a bit towards the center, to make it hang more natural
			getParticle(0+i, 0)->makeUnmovable(); 

			getParticle(0+i ,0)->offsetPos(Vec3(-0.5,0.0,0.0)); // moving the particle a bit towards the center, to make it hang more natural
			getParticle(0, num_particles_height-1-i)->makeUnmovable();
		}
	}

	//Colored in the form of a triangular mesh
	void drawShaded()
	{
		// reset normals (which where written to last frame)
		std::vector<Mass>::iterator particle;
		for(particle = particles.begin(); particle != particles.end(); particle++)
		{
			(*particle).resetNormal();
		}

		//create smooth per particle normals by adding up all the (hard) triangle normals that each particle is part of
		for(int x = 0; x<num_particles_width-1; x++)
		{
			for(int y=0; y<num_particles_height-1; y++)
			{
				//Calculating triangle normal for triangle and adding to all particles
                Vec3 normal = calcTriangleNormal(getParticle(x+1,y),getParticle(x,y),getParticle(x,y+1));
				getParticle(x+1,y)->addToNormal(normal);
				getParticle(x,y)->addToNormal(normal);
				getParticle(x,y+1)->addToNormal(normal);

				normal = calcTriangleNormal(getParticle(x+1,y+1),getParticle(x+1,y),getParticle(x,y+1));
				getParticle(x+1,y+1)->addToNormal(normal);
				getParticle(x+1,y)->addToNormal(normal);
				getParticle(x,y+1)->addToNormal(normal);
			}
		}

		
        //Drawing the flag
        for(int y=0; y<num_particles_height-1; y++)
        {
			
			for(int x = 0; x<num_particles_width-1; x++)
            {
				Vec3 color(0,0,0);
				int sec = num_particles_height/3;
				if (y < sec) // red and white color is interleaved according to which column number
					color = Vec3(1, 0.270588, 0);
				else if(y < (2*sec))
					color = Vec3(1.0f,1.0f,1.0f);
	            else if(y < (3*sec))
                     color = Vec3(0.0745098f, 0.533f, 0.0313725f);
                
 				glBegin(GL_TRIANGLES);
                drawTriangle(getParticle(x,y+1),getParticle(x,y),getParticle(x+1,y),color);
				drawTriangle(getParticle(x+1,y+1),getParticle(x,y+1),getParticle(x+1,y),color);
				glEnd();
			}
		}
		
		
		
     }

	//Function that progresses timestep
	void timeStep()
	{
		std::vector<Spring>::iterator ic;
		for(int i=0; i<SPRING_ITERATIONS; i++) // iterate over all springs several times
		{
			for(ic = springs.begin(); ic != springs.end(); ic++ )
			{
				(*ic).satisfySpring(); // satisfy constraint.
			}
		}

		std::vector<Mass>::iterator particle;
		for(particle = particles.begin(); particle != particles.end(); particle++)
		{
			(*particle).timeStep(); // calculate the position of each particle at the next time step.
		}
	}

	//Add gravitational force
	void addForce(const Vec3 direction)
	{
		std::vector<Mass>::iterator particle;
		for(particle = particles.begin(); particle != particles.end(); particle++)
		{
			(*particle).addForce(direction); // add the forces to each particle
		}

	}

	
	void windForce(const Vec3 direction)
	{
		for(int x = 0; x<num_particles_width-1; x++)
		{
			for(int y=0; y<num_particles_height-1; y++)
			{
				addWindForcesForTriangle(getParticle(x+1,y),getParticle(x,y),getParticle(x,y+1),direction);
				addWindForcesForTriangle(getParticle(x+1,y+1),getParticle(x+1,y),getParticle(x,y+1),direction);
			}
		}
	}

	void ballCollision(const Vec3 center,const float radius )
	{
		std::vector<Mass>::iterator particle;
		for(particle = particles.begin(); particle != particles.end(); particle++)
		{
			Vec3 v = (*particle).getPos()-center;
			float l = v.length();
			if ( v.length() < radius) // if the particle is inside the ball
			{
				(*particle).offsetPos(v.normalized()*(radius-l)); // project the particle to the surface of the ball
			}
		}
	}

};

// Cloth, Ball
Cloth cloth1(15,10,59,45); // one Cloth object of the Cloth class
Vec3 ball_pos(0,-5,0); // the center of our one ball
float ball_radius = 2; // the radius of our one ball

GLuint _textureId1, _textureId2; //The id of the textur
GLUquadric *quad;

float grotate = 160;
void update(int value)
{
    grotate+=2.0f;
    if(grotate>360.f)
    {
        grotate-=360;
    }
    glutPostRedisplay();
    glutTimerFunc(25,update,0);
}

GLuint loadTexture(Image* image) {
	GLuint textureId;
	glGenTextures(1, &textureId);            //Create Texture
	glBindTexture(GL_TEXTURE_2D, textureId); //Tell OpenGL which texture to edit
	//Map the image to the texture
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image->width, image->height, 0, GL_RGB, GL_UNSIGNED_BYTE, image->pixels); //The actual pixel data
	return textureId; //Returns the id of the texture
}

void init()
{
	glShadeModel(GL_SMOOTH);
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);				
	glClearDepth(1.0f);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glEnable(GL_NORMALIZE);
	glEnable(GL_COLOR_MATERIAL);
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
	
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	GLfloat lightPos[4] = {-1.0,1.0,0.5,0.0};
	glLightfv(GL_LIGHT0,GL_POSITION,(GLfloat *) &lightPos);

	glEnable(GL_LIGHT1);

	GLfloat lightAmbient1[4] = {0.0,0.0,0.0,0.0};
	GLfloat lightPos1[4] = {1.0,0.0,-0.2,0.0};
	GLfloat lightDiffuse1[4] = {0.5,0.5,0.3,0.0};

	glLightfv(GL_LIGHT1,GL_POSITION,(GLfloat *) &lightPos1);
	glLightfv(GL_LIGHT1,GL_AMBIENT,(GLfloat *) &lightAmbient1);
	glLightfv(GL_LIGHT1,GL_DIFFUSE,(GLfloat *) &lightDiffuse1);

	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,GL_TRUE);
	
	quad = gluNewQuadric();

	Image* image1 = loadBMP("earth.bmp");
	_textureId1 = loadTexture(image1);
	delete image1;
	
	Image* image2 = loadBMP("space.bmp");
	_textureId2 = loadTexture(image2);
	delete image2;
}

void drawBackground(void) {

glMatrixMode(GL_PROJECTION);
glPushMatrix();
glLoadIdentity();
glOrtho(0.0, glutGet(GLUT_WINDOW_WIDTH), 0.0, glutGet(GLUT_WINDOW_HEIGHT), -1.0, 1.0);
glMatrixMode(GL_MODELVIEW);
glPushMatrix();


glLoadIdentity();
glDisable(GL_LIGHTING);


glColor3f(1,1,1);
glEnable(GL_TEXTURE_2D);
glDepthMask( false );
glBindTexture(GL_TEXTURE_2D, _textureId2);


// Draw a textured quad
glBegin(GL_QUADS);
glTexCoord2f(0, 0); glVertex3f(0, 0, 0);
glTexCoord2f(0, 1); glVertex3f(0, glutGet(GLUT_WINDOW_HEIGHT), 0);
glTexCoord2f(1, 1); glVertex3f(glutGet(GLUT_WINDOW_WIDTH), glutGet(GLUT_WINDOW_HEIGHT), 0);
glTexCoord2f(1, 0); glVertex3f(glutGet(GLUT_WINDOW_WIDTH), 0, 0);
glEnd();


glDisable(GL_TEXTURE_2D);
glDepthMask( true );
glPopMatrix();


glMatrixMode(GL_PROJECTION);
glPopMatrix();

glMatrixMode(GL_MODELVIEW);

}

float ball_time = 0; //Counter for used to calculate the z position of the ball below

//Display function
void display(void)
{
	glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
	ball_time++;
	ball_pos.f[0] = sin(ball_time/50.0)*13;
	ball_pos.f[2] = cos(ball_time/50.0)*12;

	cloth1.addForce(Vec3(0.08,0.0,0)*TIME_STEPSIZE2); // add gravity each frame, pointing down
	cloth1.windForce(Vec3(0.5,0.1,0.2)*TIME_STEPSIZE2); // generate some wind each frame
	cloth1.timeStep(); // calculate the particle positions of the next frame
	cloth1.ballCollision(ball_pos,ball_radius); // resolve collision with the ball

	// drawing
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

	glEnable(GL_TEXTURE_2D);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    drawBackground();
 
	glEnable(GL_LIGHTING);

	glTranslatef(-10,6,-13); // move camera out and center on the cloth
	glRotatef(-5,0,1,0); // rotate a bit to see the cloth from the side
	cloth1.drawShaded(); // finally draw the cloth with smooth shading
	
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, _textureId1);
 
 	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	
	glPushMatrix();
	glTranslatef(0, 0.5, 0);
	glColor3f(0.875, 0.87109375, 0.858824);
	glutSolidSphere(0.3, 50, 50);
    GLUquadricObj *quadratic;
    glRotatef(90.0f, 1.0f, 0.0f, 0.0f);
    quadratic = gluNewQuadric();
    gluCylinder(quadratic,0.1f,0.1f,19.0f,32,32);
    glPopMatrix();
	
	glPushMatrix(); // to draw the ball we use glutSolidSphere, and need to draw the sphere at the position of the ball
	glTranslatef(ball_pos.f[0],ball_pos.f[1],ball_pos.f[2]); // hence the translation of the sphere onto the ball position
	glColor3f(1, 1, 1);
    glRotatef(90, 1.0f, 0.0f, 0.0f);
	glRotatef(grotate, 0.0f, 0.0f, 1.0f);
	gluQuadricTexture(quad, 1);
    gluSphere(quad,ball_radius-0.1,50,50);//glColor3f(0.4f,0.8f,0.5f);
	//glutSolidSphere(ball_radius-0.1,50,50); // draw the ball, but with a slightly lower radius, otherwise we could get ugly visual artifacts of cloth penetrating the ball slightly
	glPopMatrix();

	glutSwapBuffers();
	glutPostRedisplay();
}

void reshape(int w, int h)  
{
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION); 
	glLoadIdentity();  
	if (h==0)  
		gluPerspective(80,(float)w,1.0,5000.0);
	else
		gluPerspective (80,( float )w /( float )h,1.0,5000.0 );
	glMatrixMode(GL_MODELVIEW);  
	glLoadIdentity(); 
}

void keyboard( unsigned char key, int x, int y ) 
{
	switch ( key ) {
	case 27:                 //Exit program 
		exit ( 0 );
		break;  
	default: 
		break;
	}
}

void arrow_keys( int a_keys, int x, int y ) 
{
	switch(a_keys) {
	case GLUT_KEY_UP:
		glutFullScreen();
		break;
	case GLUT_KEY_DOWN: 
		glutReshapeWindow (1280, 720 );
		break;
	default:
		break;
	}
}

int main ( int argc, char** argv ) 
{
	glutInit( &argc, argv );

	glutInitDisplayMode( GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH ); 
	glutInitWindowSize(1280, 720 ); 

	glutCreateWindow( "Vande Mataram" );
	init();
	glutDisplayFunc(display);  
	glutReshapeFunc(reshape);
	glutKeyboardFunc(keyboard);
	glutSpecialFunc(arrow_keys);
	glutTimerFunc(25,update,0);

	glutMainLoop();
}
