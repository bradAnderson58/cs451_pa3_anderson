#include "Draw.hpp"
#include "Skeleton.hpp"
#include <GL/gli.h>

#include <iostream>
#include <cassert>
using std::cout;
using std::endl;

namespace {
	void draw_arrow(float len) {
		float h = 0.15f / 0.45f * len; //head size.
		float s = len - h; //shaft length
		glBegin(GL_TRIANGLE_STRIP);
		glVertex3f( 0.0f,  0.0f,-0.05f);
		glVertex3f(-0.05f, 0.0f, 0.0f);
		glVertex3f( 0.05f, 0.0f, 0.0f);
		glVertex3f(-0.05f - 0.02f / 0.15f * h, 0.0f, s);
		glVertex3f( 0.05f + 0.02f / 0.15f * h, 0.0f, s);
		glVertex3f(-0.05f - 0.13f / 0.15f * h, 0.0f, s);
		glVertex3f( 0.05f + 0.13f / 0.15f * h, 0.0f, s);
		glVertex3f( 0.0f,  0.0f, s + h);
		glEnd();
	}
}

inline void glTranslate(const Vector3d& v)
{
	glTranslated(v[0],v[1],v[2]);
}

inline void quat_to_gl_matrix(Quaternion const &a, double *mat) 
{
	Matrix3x3 m3x3 = a.getMatrix();

	double res[16] =
	{
		m3x3[0][0], m3x3[1][0], m3x3[2][0], 0,
		m3x3[0][1], m3x3[1][1], m3x3[2][1], 0,
		m3x3[0][2], m3x3[1][2], m3x3[2][2], 0,
		0, 0, 0, 1
	};

	for (unsigned int i = 0; i < 16; ++i)  mat[i] = res[i]; 
}


inline void glRotate(const Quaternion& q)
{
	q.getMatrix();
	double mat[16];
	quat_to_gl_matrix(q, mat);
	glMultMatrixd(mat);
}

void Character::draw(Pose const &pose, State const &state, bool detail, bool color, bool select, float alpha) 
{

	glPushMatrix();
	glTranslate(state.position);
	glRotatef(state.orientation * 180.0f / (float)M_PI, 0.0f, 1.0f, 0.0f);
	glTranslate(pose.root_position);
	glRotate(pose.root_orientation);

	vector< int > parent_stack;
	
	parent_stack.push_back(-1);
	for (unsigned int b = 0; b < pose.bone_orientations.size(); ++b) 
	{
		while(!parent_stack.empty() && parent_stack.back() != pose.skeleton->bones[b].parent) {
			glPopMatrix();
			parent_stack.pop_back();
		}
		assert(!parent_stack.empty());

		glPushMatrix();
		glRotate(pose.bone_orientations[b]);

		if (select) glPushName(b);
//----- pasted from eariler skeleton draw code -----
	//draw thick bits.
	{
		static GLUquadric *quad = NULL;
		if (quad == NULL) {
			quad = gluNewQuadric();
		}

		Vector3d t = pose.skeleton->bones[b].direction;
		Vector3d t2;
		if (t[0] == t[1] && t[1] == t[2]) {
			t2 = (t% Vector3d(t[1], t[2], -t[0])).normalize();
		} else {
			t2 = ((t % Vector3d(t[1], t[2], t[0]))).normalize();
		}
		Vector3d t3 = (t2 % t);
		glPushMatrix();
		double mat[16] = {
			t2[0], t2[1], t2[2], 0,
			t3[0], t3[1], t3[2], 0,
			t[0], t[1], t[2], 0,
			0,0,0,1
		};
		glMultMatrixd(mat);
		if (color) {
			glColor4f(pose.skeleton->bones[b].color[0] * 0.8, pose.skeleton->bones[b].color[1] * 0.8, pose.skeleton->bones[b].color[2] * 0.8, alpha);
		}
		gluCylinder(quad, pose.skeleton->bones[b].radius, pose.skeleton->bones[b].radius, pose.skeleton->bones[b].length, detail?16:8, 1);
		glRotated(180,1,0,0);
		gluDisk(quad, 0, pose.skeleton->bones[b].radius, detail?16:8, 1);
		glRotated(180,1,0,0);
		glTranslated(0, 0, pose.skeleton->bones[b].length);
		gluDisk(quad, 0, pose.skeleton->bones[b].radius, detail?16:8, 1);
		glTranslated(-pose.skeleton->bones[b].radius, 0, -pose.skeleton->bones[b].length);

		if (detail) {
			if (color) {
				glColor4f(pose.skeleton->bones[b].color[0], pose.skeleton->bones[b].color[1], pose.skeleton->bones[b].color[2], alpha);
			}
			gluCylinder(quad, pose.skeleton->bones[b].radius * 0.05, pose.skeleton->bones[b].radius * 0.05, pose.skeleton->bones[b].length, 6, 1);
		}
		glPopMatrix();
	}

//----- end paste -----
		if (select) glPopName();


		glTranslate(pose.skeleton->bones[b].direction * pose.skeleton->bones[b].length);
		parent_stack.push_back(b);
	} // end for
	while (!parent_stack.empty()) {
		glPopMatrix();
		parent_stack.pop_back();
	}
}

void Character::draw(Control const &control, State const &state) {
	glPushMatrix();
		glTranslate(state.position);
		glRotatef(state.orientation * 180.0f / (float)M_PI, 0.0f, 1.0f, 0.0f);
		float ang = atan2(control.desired_velocity[0], control.desired_velocity[1]);
		glRotatef(ang / (float)M_PI * 180.0f, 0.0f, 1.0f, 0.0f);

		glColor3f(1.0f,0.3f,1.0f);
		draw_arrow((control.desired_velocity.norm()));
	glPopMatrix();

}

void Character::draw(State const &state) 
{
	glPushMatrix();
		glTranslate(state.position + Vector3d(0.0f, 0.1f, 0.0f));
		glRotatef(state.orientation * 180.0f / (float)M_PI, 0.0f, 1.0f, 0.0f);

		//glColor3f(1,1,0.3);
		draw_arrow(0.45f);
	glPopMatrix();
}

