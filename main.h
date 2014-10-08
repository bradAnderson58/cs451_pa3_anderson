//------------------------------------------------------------------------------
//  Copyright 2007-2014 by Jyh-Ming Lien and George Mason University
//  See the file "LICENSE" for more information
//------------------------------------------------------------------------------


#pragma once

#include "objReader.h"
#include "model.h"
#include "Library.hpp" //motion capture library
#include "pose_utils.hpp"
#include <list>
#include <float.h>
#include <ctime>
#include <cstdlib>
using namespace std;


//-----------------------------------------------------------------------------
// INPUTS
list<string> input_filenames;
string mocap_dir; //motion capture dir

//-----------------------------------------------------------------------------
// Intermediate data
list<model> models; //NOTE: only the first model of the list is used in this code
float R=0;          //radius
Point3d COM;        //center of mass

int currentMotion=0; //current motion dataset, used to animate skeleton
int currentFrame=0;  //current frame in the dataset, used to animate skeleton

//-----------------------------------------------------------------------------

/////------ PA3 Variables START-----

bool BindToSkin = false;       // initially the skeleton is not binded to the skin (mesh)

Character::Pose BindingPose;   // at this pose that the skeleton binds to the skin

vector< vector<double> > SkinningWeights; //weights for each vertex and each bone
                                          //there are N vector<double>, where N is the number of vertices in the mesh
                                          //there are K double in vector<double>, where K is the number of bones in skeleton 

vector< vector<Vector3d> > BoneSpaceCoordinates; //local coordinates for each vertex in bone subspace
                                                //there are N vector<Point3d>, where N is the number of vertices in the mesh
                                                //there are K double in vector<Point3d>, where K is the number of bones in skeleton 
Character::WorldBones *out1;		//world bones
////------ PA3 Variables END-----

//-----------------------------------------------------------------------------

/////------ PA3 TODOs START-----

//TODO: implement this function to setup the binding pose.
//      See details below
void setupBindingPose();

//TODO: implement this function to bind the skeleton to the skin.
//      See details below
void bind2skin();

//TODO: skeleton-subspace deformation. perform SSD 
void SSD();

/////------ PA3 TODOs END-----

//-----------------------------------------------------------------------------
bool readfromfile();
void computeCOM_R();

//-----------------------------------------------------------------------------
bool parseArg(int argc, char ** argv)
{
    for(int i=1;i<argc;i++){
        if(argv[i][0]=='-')
        {
			if (string(argv[i]) == "-mocap")      mocap_dir = argv[++i];
			else
				return false; //unknown
        }
        else{
            input_filenames.push_back(argv[i]);
        }
    }

    return true;
}

void printUsage(char * name)
{
    //int offset=20;
    cerr<<"Usage: "<<name<<" [options] -mocap dir *.obj \n"
        <<"options:\n\n";
    cerr<<"\n-- Report bugs to: Jyh-Ming Lien jmlien@gmu.edu"<<endl;
}

//-----------------------------------------------------------------------------

bool readfromfiles()
{
	if (input_filenames.empty())
	{
		cerr << "! Error: No input model" << endl;
		return false;
	}

	if (mocap_dir.empty())
	{
		cerr << "! Error: No input motion capture data" << endl;
		return false;
	}

	//read obj model
    long vsize=0;
    long fsize=0;

    uint id=0;
    for(list<string>::iterator i=input_filenames.begin();i!=input_filenames.end();i++,id++){
        cout<<"- ["<<id<<"/"<<input_filenames.size()<<"] Start reading "<<*i<<endl;
        model m;
        if(!m.build(*i)) continue;
        cout<<"- ["<<id<<"/"<<input_filenames.size()<<"] Done reading "<<m.v_size<<" vertices and "<<m.t_size<<" facets"<<endl;
        vsize+=m.v_size;
        fsize+=m.t_size;
        models.push_back(m);
    }
    cout<<"- Total: "<<vsize<<" vertices, "<<fsize<<" triangles, and "<<input_filenames.size()<<" models"<<endl;
    computeCOM_R();
	
	//read mocap skeleton and animation
	Library::init(mocap_dir);

	//setup binding pose
	setupBindingPose();

    return true;
}

void computeCOM_R()
{
    //compute a bbox
    double box[6]={FLT_MAX,-FLT_MAX,FLT_MAX,-FLT_MAX,FLT_MAX,-FLT_MAX};
    //-------------------------------------------------------------------------
    for(list<model>::iterator i=models.begin();i!=models.end();i++){
        for(unsigned int j=0;j<i->v_size;j++){
            Point3d& p=i->vertices[j].p;
            if(p[0]<box[0]) box[0]=p[0];
            if(p[0]>box[1]) box[1]=p[0];
            if(p[1]<box[2]) box[2]=p[1];
            if(p[1]>box[3]) box[3]=p[1];
            if(p[2]<box[4]) box[4]=p[2];
            if(p[2]>box[5]) box[5]=p[2];
        }//j
    }//i

    //-------------------------------------------------------------------------
    // compute center of mass and R...
    COM.set( (box[1]+box[0])/2,(box[3]+box[2])/2,(box[5]+box[4])/2);

    //-------------------------------------------------------------------------
	R=0;
    for(list<model>::iterator i=models.begin();i!=models.end();i++){
        for(unsigned int j=0;j<i->v_size;j++){
            Point3d& p=i->vertices[j].p;
            float d=(float)(p-COM).normsqr();
            if(d>R) R=d;
        }//j
    }//i

    R=sqrt(R);
}

//I wrote this so I could put degrees into setupBinding Pose method
double toRadians(double degree){
	return degree * (PI / 180);
}

//I wrote this to encapsulate distance between two points
double pointDistance(Vector3d start, Vector3d end){
	double xd = abs(end[0] - start[0]);
	double yd = abs(end[1] - start[1]);
	double zd = abs(end[2] - start[2]);
	return sqrt(xd*xd + yd*yd + zd*zd);
}

void setupBindingPose()
{
	//set binding pose to zero pose
	Library::Motion const &mo = Library::motion(0);
	mo.get_pose(0, BindingPose);

	BindingPose.root_position = Vector3d(0, 0, 0);
	BindingPose.root_orientation = Quaternion();
	for (int k = 0; k<BindingPose.bone_orientations.size(); k++) 
		BindingPose.bone_orientations[k] = Quaternion();

	//
	// TODO: Determine the Binding Pose, you can do it manually 
	//       or you can build a GUI to help you determine the binding pose
	//       The binding pose is a pose that matches the input mesh
	//

	//Move the whole root down a bit
	BindingPose.root_position[1] -= .5;

	//Hard code them bones ---------------------------------------------------------------------
	BindingPose.bone_orientations[10] = Quaternion::get(toRadians(-35), Vector3d(0, 1, 0)) * Quaternion::get(toRadians(-5), Vector3d(0, 0, 1));
	BindingPose.bone_orientations[18] = Quaternion::get(toRadians(-10), Vector3d(0, 1, 0));
	BindingPose.bone_orientations[19] = Quaternion::get(toRadians(10), Vector3d(0, 1, 0));
	BindingPose.bone_orientations[20] = Quaternion::get(toRadians(35), Vector3d(0, 1, 0));
	BindingPose.bone_orientations[26] = Quaternion::get(toRadians(15), Vector3d(0, 0, 1));
	BindingPose.bone_orientations[1] = Quaternion::get(toRadians(-15), Vector3d(0, 0, 1));
	//------------------------------------------------------------------------------------------
}
//
// This function will be called when "B" (captial b) is pressed
//
void bind2skin()
{
	if (BindToSkin) return; //already binded
	
	//work on the first model only
	model& model = models.front();

	//
	// TODO: compute SkinningWeights using BindingPose **DONE?
	//
	//       Read amc_library/Lirary.hpp, amc_library/Skeleton.hpp
	//       to find out how to get all the bones
	//

	//get a worldBones object that we can use to determine vertex closeness
	out1 = new Character::WorldBones();
	Character::get_world_bones(BindingPose, *out1);
	
	//vectors for math formula
	Vector3d p_a = Vector3d();
	Vector3d a_b = Vector3d();
	Vector3d p = Vector3d();

	//used for skinningWeights
	double bone_dist;
	double sum = 0;
	
	//Loop through each vertex in the model
	for (int i = 0; i < model.v_size; i++){
		vertex & v = model.vertices[i];
		p[0] = v.p[0];
		p[1] = v.p[1];
		p[2] = v.p[2];

		SkinningWeights.push_back(vector<double>());

		//Get the distance from each bone (30)
		for (int b = 0; b < 30; b++){

			//get neccessary variables
			a_b = out1->tips[b] - out1->bases[b];
			double m_ab = a_b * a_b;
			p_a = p - out1->bases[b];

			double m_ap = (p_a * a_b) / m_ab;

			//bone-distance is the distance from this vertex point to the current base
			if (m_ap < 0) bone_dist = pointDistance(p, out1->bases[b]);
			
			//bone-distance is to current tip
			else if (m_ap > 1) bone_dist = pointDistance(p , out1->tips[b]);
			
			//bone distance is to the p'
			else bone_dist = pointDistance(p, out1->bases[b] + (m_ap * a_b));

			//Now we push the bone_dist on the weights, which we will process later
			SkinningWeights[i].push_back(bone_dist);
			sum += (1 / bone_dist);

		}//end bone loop
		
		Point3d best = Point3d(-1,-1,-1);
		Point3d bestval = Point3d(-1,-1,-1);

		//now do weight calcs 30 bones
		for (int w = 0; w < 30; w++){
			
			//If the current is better than the best, or the first one
			if ((SkinningWeights[i][w] < bestval[0] || bestval[0] == -1)){
				best[2] = best[1];
				bestval[2] = bestval[1];
				best[1] = best[0];
				bestval[1] = bestval[0];
				best[0] = w;
				bestval[0] = SkinningWeights[i][w];
				SkinningWeights[i][w] = 0;
			}

			//If the current is better than the second, or the second one
			else if ((SkinningWeights[i][w] < bestval[1] || bestval[1] == -1)){
				best[2] = best[1];
				bestval[2] = bestval[1];
				best[1] = w;
				bestval[1] = SkinningWeights[i][w];
			}

			//If the current is better than the third, or the third
			else if ((SkinningWeights[i][w] < bestval[2] || bestval[2] == -1)){
				best[2] = w;
				bestval[2] = SkinningWeights[i][w];
			}

			//All other weights are 0
			SkinningWeights[i][w] = 0;
			
		}//end weight loop
		
		//Case where only one bone is close enough
		if (bestval[1] > .5 && bestval[2] > .5){
			SkinningWeights[i][best[0]] = 1;
			
		}

		//Two bones are close enough
		else if (bestval[2] > .5){
			sum = ((1 / bestval[0]) + 1 / bestval[1]);
			SkinningWeights[i][best[0]] = (1 / bestval[0]) / sum;
			SkinningWeights[i][best[1]] = (1 / bestval[1]) / sum;
			
		}

		//All three are close enough
		else{
			sum = (1 / bestval[0]) + (1 / bestval[1]) + (1 / bestval[2]);
			SkinningWeights[i][best[0]] = (1 / bestval[0]) / sum;
			SkinningWeights[i][best[1]] = (1 / bestval[1]) / sum;
			SkinningWeights[i][best[2]] = (1 / bestval[2]) / sum;
			
		}
		
	}//end vertex loop
	
	//
	// TODO: compute BoneSpaceCoordinates using BindingPose  **DONE?
	//       
	//       determine the cooridnates of each model vertex with respect to each bone
	//       in binding pose

	Quaternion qw;
	Vector3d prime;

	//For each model we must also add a bone space coordinate
	for (int i = 0; i < model.v_size; i++){
		BoneSpaceCoordinates.push_back(vector<Vector3d>());
		vertex & v = model.vertices[i];
		p[0] = v.p[0];
		p[1] = v.p[1];
		p[2] = v.p[2];

		for (int b = 0; b < 30; b++){

			//Get the inverse of the orientation, rotate by point - base
			qw = -(out1->orientations[b]);
			prime = (qw.rotate(p - out1->bases[b]));
			
			BoneSpaceCoordinates[i].push_back(prime);
		}//end bone loop
	}//end vertex loop
	

	BindToSkin = true;
	
}

//TODO: skeleton-subspace deformation. perform SSD 
void SSD()
{
	//
	//work on the first model only
	//
	model& model = models.front();

	//
	// recompute the position of each vertex in model
	// using BoneSpaceCoordinates and SkinningWeights
	//

	//Use this to get current Pose and WorldBones
	Library::Motion const &mo = Library::motion(currentMotion);
	Character::Pose pos;
	mo.get_pose(currentFrame, pos);
	Character::get_world_bones(pos, *out1);

	//these
	Vector3d temp = Vector3d();
	Vector3d temp2 = Vector3d();

	for (int i = 0; i < model.v_size; i++){

		temp[0] = 0;
		temp[1] = 0;
		temp[2] = 0;

		for (int b = 0; b < 30; b++){

			//Order matters
			Vector3d thing = out1->bases[b] + (out1->orientations[b].rotate(BoneSpaceCoordinates[i][b]));
			temp2 = (SkinningWeights[i][b] * thing);
			temp[0] += temp2[0];
			temp[1] += temp2[1];
			temp[2] += temp2[2];
		}

		model.vertices[i].p[0] = temp[0];
		model.vertices[i].p[1] = temp[1];
		model.vertices[i].p[2] = temp[2];

	}

}

//-----------------------------------------------------------------------------
//
//
//
//  Open GL stuff below
//
//
//-----------------------------------------------------------------------------

#include <draw.h>



