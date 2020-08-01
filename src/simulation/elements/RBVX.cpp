#include "simulation/ElementCommon.h"
#include "simulation/ElementRigidbody.h"
#include <math.h>
#include <stdio.h>

static int RBVX_UPDATE(UPDATE_FUNC_ARGS);
static int RBVX_GRAPHICS(GRAPHICS_FUNC_ARGS);
static void RBVX_CREATE(ELEMENT_CREATE_FUNC_ARGS);

/*
Tells whether a point lies inside a rigidbody object.
*/
char in_obj(Simulation* sim, int centre_id,unsigned int px,unsigned int py)
{
	if(RBSY_VERTICES(centre_id,sim->parts)<3)return 0;
	double cx,cy;
	unsigned int current_vx=RBSY_VXID(sim->parts[centre_id]),next_vx=RBSY_VXID(sim->parts[current_vx]),origin=current_vx;
	int x1,x2,y1,y2;
	char sign;
	/*The first edge determines the sign we are expecting*/
	cx=sim->parts[current_vx].x;cy=sim->parts[current_vx].y;
	x1=px-cx;y1=py-cy;
	x2=sim->parts[next_vx].x-cx;y2=sim->parts[next_vx].y-cy;
	sign=((x1*y2)-(x2*y1))>0;
	current_vx=next_vx;
	/*Then we keep going forwards until we come back to the origin*/
	while(current_vx!=origin)
	{
		next_vx=RBSY_VXID(sim->parts[current_vx]);
		if(next_vx==RBSY_NONE)next_vx=origin;
		cx=sim->parts[current_vx].x;cy=sim->parts[current_vx].y;
		x1=px-cx;y1=py-cy;
		x2=sim->parts[next_vx].x-cx;y2=sim->parts[next_vx].y-cy;
		if(sign!=((x1*y2)-(x2*y1))>0)return 0;
		current_vx=next_vx;
	}
	return 1;
}

void Element::Element_RBVX()
{
	Identifier = "DEFAULT_PT_RBVX";
	Name = "RBVX";
	Colour = PIXPACK(0xCAAFEE);
	MenuVisible = 1;
	MenuSection = SC_SPECIAL;
	Enabled = 1;

	Description = "A rigidbody vertex. Binds to the newest RBCN as a new vertex.";

	Update = &RBVX_UPDATE;
	Graphics = &RBVX_GRAPHICS;
	Create = &RBVX_CREATE;
}

static void RBVX_CREATE(ELEMENT_CREATE_FUNC_ARGS)
{
	RBSY_SET_VXID(sim->parts[i],RBSY_NONE);
	if(!sim->sys_pause)return;
	/*Firstly we check a 101x101 square around the particle origin to search for a RBCN particle*/
	/*We'll search for the RBCN particle with the highest ID (generally, the newest one)*/
	unsigned int xx=x>=50?x-50:0,yy=y>=50?y-50:0,maxid=0,id;
	char found=0;
	for(;yy<y+51;yy++)
	{
		for(xx=x>=50?x-50:0;xx<x+51;xx++)
		{
			if(TYP(sim->pmap[yy][xx])==PT_RBCN)
			{
				found=1;
				id=ID(sim->pmap[yy][xx]);
				maxid=id>maxid?id:maxid;
			}
		}
	}
	if(!found)
	{
		sim->kill_part(i);
		return;
	}
	/*Then we add ourselves to the vertex list*/
	unsigned int lv=RBSY_LAST_VERTEX(maxid,sim->parts);
	RBSY_SET_VXID(sim->parts[lv],i);
	/*And recalculate the centre's position*/
	double avx=0,avy=0;
	unsigned int current_vx=RBSY_VXID(sim->parts[maxid]),l=RBSY_VERTICES(maxid,sim->parts);
	while(current_vx!=RBSY_NONE)
	{
		avx+=sim->parts[current_vx].x;
		avy+=sim->parts[current_vx].y;
		current_vx=RBSY_VXID(sim->parts[current_vx]);
	}
	if(l>2)
	{
		avx/=l;
		sim->parts[maxid].x=avx;
		avy/=l;
		sim->parts[maxid].y=avy;
	}
	printf("avx,avy,l = %lf,%lf,%d\n",avx,avy,l);
	/*And its vertices' dist and ang values*/
	int dx,dy;
	uint16_t dist_v;
	current_vx=RBSY_VXID(sim->parts[maxid]);
	while(current_vx!=RBSY_NONE)
	{
		dx=sim->parts[current_vx].x-sim->parts[maxid].x;
		dy=sim->parts[current_vx].y-sim->parts[maxid].y;
		dist_v=sqrt((dx*dx)+(dy*dy));
		RBSY_SET_DIST(sim->parts[current_vx],ceil(dist_v));
		RBSY_SET_ANGL(sim->parts[current_vx],atan2(dy,dx));
		printf("Edge at distance %hu, angle %.2f\n(dx,dy)=(%d,%d)\n",dist_v,atan2(dy,dx),dx,dy);
		current_vx=RBSY_VXID(sim->parts[current_vx]);
	}
	/*Then we reset the centre's angle value*/
	RBSY_SET_ANGL(sim->parts[maxid],0.0);

	/*And calculate the moment of inertia*/
	/*For that, firstly we estabilish the bounding box*/
	unsigned int lx=XRES,ly=YRES,mx=0,my=0,cx,cy;
	uint16_t moin=0;
	current_vx=RBSY_VXID(sim->parts[maxid]);
	while(current_vx!=RBSY_NONE)
	{
		cx=sim->parts[current_vx].x;
		cy=sim->parts[current_vx].y;
		lx=cx<lx?cx:lx;
		ly=cy<ly?cy:ly;
		mx=cx>mx?cx:mx;
		my=cy>my?cy:my;
		current_vx=RBSY_VXID(sim->parts[current_vx]);
	}
	/*Then for every pixel that's in the polygon, we make add 1 to the inertia*/
	for(xx=lx;xx<mx;xx++)
	{
		for(yy=ly;yy<my;yy++)
		{
			if(in_obj(sim,maxid,xx,yy))
				moin++;
		}
	}
	current_vx=RBSY_VXID(sim->parts[maxid]);
	RBSY_SET_ROIN(sim->parts[current_vx],moin);
}

static int RBVX_UPDATE(UPDATE_FUNC_ARGS)
{
	return 0;
}

static int RBVX_GRAPHICS(GRAPHICS_FUNC_ARGS)
{
	return 0;
}

