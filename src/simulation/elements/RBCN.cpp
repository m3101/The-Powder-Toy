#include "simulation/ElementCommon.h"
#include "simulation/ElementRigidbody.h"
#include <math.h>
#include <cfloat>

/*
Rigidbody centre of mass/controller.
This element defines a separate rigidbody object.
All rigidbody vertices added afterwards will be associated with this object unless
another center is added (then they will be added to that center and so forth).
Made with <3 by Amélia O. F. da S.
Github: https://github.com/m3101
I hope you enjoy it!
*/

/*
Tells whether a point lies inside a polygon/rigidbody object.
*/
char in_polygon(Simulation* sim, int centre_id,unsigned int px,unsigned int py)
{
	if(RBSY_VERTICES(centre_id,sim->parts)<3)return 0;
	double cx,cy;
	unsigned int current_vx=RBSY_VXID(sim->parts[centre_id]),next_vx,origin=current_vx;
	int x1,x2,y1,y2;
	do
	{
		next_vx=RBSY_VXID(sim->parts[current_vx]);
		if(next_vx==RBSY_NONE)next_vx=origin;
		cx=sim->parts[current_vx].x;cy=sim->parts[current_vx].y;
		x1=px-cx;y1=py-cy;
		x2=sim->parts[next_vx].x-cx;y2=sim->parts[next_vx].y-cy;
		if(((x1*y2)-(x2*y1))<0)return 0;
		current_vx=next_vx;
	}
	while(current_vx!=origin);
	return 1;
}

/*
Returns the closest edge (1-indexed) if the point is in the polygon.
Otherwise, returns 0.
*/
char in_polygon_idx(Simulation* sim, int centre_id,unsigned int px,unsigned int py)
{
	if(RBSY_VERTICES(centre_id,sim->parts)<3)return 0;
	double cx,cy;
	unsigned int current_vx=RBSY_VXID(sim->parts[centre_id]),next_vx,origin=current_vx;
	float x1,x2;/*vector 1*/
	float y1,y2;/*vector 2*/
	float cpx,cpy;/*closest point from (px,py) to the edge*/
	float lincx,lincy;/*vector "lin" (from the cp to the point)*/
	double dist,min=DBL_MAX;
	/*
	proj-ratio is the <u,v>/<v,v> ratio on projv(u)
	(see https://en.wikipedia.org/wiki/Gram%E2%80%93Schmidt_process 
	for a better understanding of these projections)
	*/
	double proj_ratio;
	char i=1,midx;
	/*For each edge,*/
	do
	{
		next_vx=RBSY_VXID(sim->parts[current_vx]);
		if(next_vx==RBSY_NONE)next_vx=origin;
		cx=sim->parts[current_vx].x;cy=sim->parts[current_vx].y;
		/*
		Calculate two vectors
		vector 1, from the first vertex of the edge to the point
		vector 2, from the first vertex of the edge to the next (aka "the edge")
		*/
		x1=px-cx;y1=py-cy;
		x2=sim->parts[next_vx].x-cx;y2=sim->parts[next_vx].y-cy;
		/*
		Compute our projection onto the edge
		*/
		proj_ratio=((x1*x2)+(y1*y2))/((x2*x2)+(y2*y2));
		/*
		If the projection lies outside the edge,
		we're sure we're not the closest one.
		*/
		if(proj_ratio<0||proj_ratio>1)
			goto next_edge;
		/*
		Otherwise, we calculate our (squared) distance
		*/
		cpx=x2*proj_ratio;
		cpy=y2*proj_ratio;
		lincx=x1-cpx;
		lincy=y1-cpy;
		dist=(lincx*lincx)+(lincy*lincy);
		if(dist<0)return 0;
		if(dist<min)
		{
			min=dist;
			midx=i;
		}
		next_edge:
		i++;
		current_vx=next_vx;
	}
	while(current_vx!=origin);
	return midx;
}

/*
Returns the components for a collision (referring to the closest edge to the collision).
These components are "how far the center has to move so this collision is no longer a thing"
long int ret:
LINR_XCOM = X linear component of the collision;
LINR_YCOM = Y linear component of the collision;
ROTA_COMP = Signed rotational component (the component orthogonal to the line from the point to the center);
[0-1-2-3-4-5-6-7-8-9-A-B-C-D-E-F-0-1-2-3-4-5-6-7-8-9-A-B-C-D-E-F-0-1-2-3-4-5-6-7-8-9-A-B-C-D-E-F-0-1-2-3-4-5-6-7-8-9-A-B-C-D-E-F]
[???????????????????????????????|INT16        LINR_XCOM         |INT16      LINR_YCOM           |INT16        ROTA_COMP         ]
*/
long int min_polygon(Simulation* sim, int centre_id,unsigned int px,unsigned int py)
{
	if(RBSY_VERTICES(centre_id,sim->parts)<3)return 0;
	double cx,cy;
	double ccx=sim->parts[centre_id].x,ccy=sim->parts[centre_id].y;
	unsigned int current_vx=RBSY_VXID(sim->parts[centre_id]),next_vx,origin=current_vx;
	float x1,x2;/*vector 1*/
	float y1,y2;/*vector 2*/
	float cpx,cpy;/*closest point from (px,py) to the edge*/
	float lincx,lincy;/*vector "lin" (from the cp to the point)*/
	uint16_t retlincx,retlincy;/*Linear components to be returned*/
	float rotx,roty;/*Rotational component*/
	uint16_t rotc;/*Signed "torque" component to be returned*/
	double dist,min=DBL_MAX;
	/*
	proj-ratio is the <u,v>/<v,v> ratio on projv(u)
	(see https://en.wikipedia.org/wiki/Gram%E2%80%93Schmidt_process 
	for a better understanding of these projections)
	*/
	double proj_ratio;
	char i=1;
	/*For each edge,*/
	do
	{
		next_vx=RBSY_VXID(sim->parts[current_vx]);
		if(next_vx==RBSY_NONE)next_vx=origin;
		cx=sim->parts[current_vx].x;cy=sim->parts[current_vx].y;
		/*
		Calculate two vectors
		vector 1, from the first vertex of the edge to the point
		vector 2, from the first vertex of the edge to the next (aka "the edge")
		*/
		x1=px-cx;y1=py-cy;
		x2=sim->parts[next_vx].x-cx;y2=sim->parts[next_vx].y-cy;
		/*
		Compute our projection onto the edge
		*/
		proj_ratio=((x1*x2)+(y1*y2))/((x2*x2)+(y2*y2));
		/*
		If the projection lies outside the edge,
		we're sure we're not the closest one.
		*/
		if(proj_ratio<0||proj_ratio>1)
			goto next_edge;
		/*
		Otherwise, we calculate our (squared) distance
		*/
		cpx=x2*proj_ratio;
		cpy=y2*proj_ratio;
		lincx=x1-cpx;
		lincy=y1-cpy;
		dist=(lincx*lincx)+(lincy*lincy);
		if(dist<0)return 0;
		if(dist<min)
		{
			/*
			If we're at the minimum distance, we store our current
			components and also calculate the rotational component.
			*/
			retlincx=lincx;retlincy=lincy;
			cx=px-ccx;cy=py-ccy;
			proj_ratio=((lincx*cx)+(lincy*cy))/((cx*cx)/(cy*cy));
			rotx=lincx-ccx*proj_ratio;
			roty=lincy-ccy*proj_ratio;
			/*
			The "torque" returned will be the length of this vector
			with the sign of the "z" component of their cross product
			in 3d (which we can use to indicate whether it is to the
			"right" or to the "left" of the vector).
			*/
			rotc=sqrt((rotx*rotx)+(roty*roty))*((rotx*cy-roty*cx)<0?-1:1);
			min=dist;
		}
		next_edge:
		i++;
		current_vx=next_vx;
	}
	while(current_vx!=origin);
	return (long int)rotc+(((long int)retlincy)<<16)+(((long int)retlincx)<<32);
}

static int RBCN_UPDATE(UPDATE_FUNC_ARGS);
static int RBCN_GRAPHICS(GRAPHICS_FUNC_ARGS);
static void RBCN_CREATE(ELEMENT_CREATE_FUNC_ARGS);

void Element::Element_RBCN()
{
	Identifier = "DEFAULT_PT_RBCN";
	Name = "RBCN";
	Colour = PIXPACK(0xFF0000);
	MenuVisible = 1;
	MenuSection = SC_SPECIAL;
	Enabled = 1;
	Description = "A separate rigidbody object. Add one of these then define its vertices with RBVX.";

	Update = &RBCN_UPDATE;
	Graphics = &RBCN_GRAPHICS;
	Create = &RBCN_CREATE;
}

static void RBCN_CREATE(ELEMENT_CREATE_FUNC_ARGS)
{
	RBSY_SET_VXID(sim->parts[i],RBSY_NONE);
	RBSY_SET_VELX(sim->parts[i],0);
	RBSY_SET_VELY(sim->parts[i],0);
	RBSY_SET_RVEL(sim->parts[i],0.0);
	RBSY_SET_CMID(sim->parts[i],i);
}

/*This constant affects the rotational inertia of the object*/
#define RBCN_ROTC 10.0
/*Dampening on collision. 1=100% bouncy. 0=Sticky*/
#define RBCN_DAMP .99

static int RBCN_UPDATE(UPDATE_FUNC_ARGS)
{
	if(RBSY_VERTICES(i,sim->parts)<3)return 0;
	/*Firstly we estabilish our bounding box, the base of all of our operations*/
	unsigned int xx,yy,lx=XRES,ly=YRES,mx=0,my=0,cx,cy,current_vx=RBSY_VXID(sim->parts[i]);
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
/*
	A cycle happens as following:
	1) At the object's centre update tick, it checks for collisions within the object
   	   and updates its rotational and directional speeds accordingly.
	2) The centre calculates the pressure updates and its influences on its own speeds.
	3) The centre updates its position and rotation according to its speeds.
	4) The centre updates its vertices' positions according to its orientation and position.
*/

	/*The first thing we should do is store the current state of the object*/
	unsigned int firstv=RBSY_VXID(sim->parts[i]);
	float rot,ang,vx,vy,fx,fy;
	float tvel,force;
	vx=RBSY_VELX(sim->parts[i]);vy=RBSY_VELY(sim->parts[i]);
	rot=RBSY_RVEL(sim->parts[i]);

	/*Phase 0 - Gravity*/
	float gx,gy;
	sim->GetGravityField(x,y,1,1,gx,gy);
	vx+=gx;
	vy+=gy;

	/*Phase 1 - Collision checking*/
	long int collision;
	short forcex,forcey,torque;
	printf("Phase 1####\n");
	for(xx=lx;xx<mx;xx++)
	{
		for(yy=ly;yy<my;yy++)
		{
			if(TYP(sim->pmap[yy][xx])&&TYP(sim->pmap[yy][xx])!=PT_RBCN&&TYP(sim->pmap[yy][xx])!=PT_RBVX)
			{
				collision=min_polygon(sim,i,xx,yy);
				if(collision!=0)
				{
					forcex=-(collision>>32)&0xFFFF;
					forcey=-(collision>>16)&0xFFFF;
					if(forcex==0||forcey==0)continue;
					torque=-collision&0xFFFF;
					/*
					Collisions should only redirect and dampen velocity
					*/
					force=sqrt((forcex*forcex)+(forcey*forcey));
					fx=forcex/force;fy=forcey/force;
					tvel=sqrt((vx*vx)+(vy*vy))*RBCN_DAMP;
					vx=fx*tvel;
					vy=fy*tvel;
					rot=(float)torque/(tvel*RBCN_ROTC*RBSY_ROIN(sim->parts[firstv]));
					printf("Collision at %d,%d! Edge number %d.\n",xx,yy,in_polygon_idx(sim,i,xx,yy));
					printf("\tResulting forces:\n\tx\ty\tt\n\t%hd\t%hd\t%hd\n",forcex,forcey,torque);
				}
			}
		}
	}

	/*Phase 2 - Pressure Updates*/

	/*Phase 3 - Updating the COM*/
	sim->parts[i].x+=vx;
	sim->parts[i].y+=vy;
	ang=RBSY_ANGL(sim->parts[i]);
	ang+=rot;
	ang=fmod(ang,RBSY_2PI);

	char tmp;
	tmp=vx;
	RBSY_SET_VELX(sim->parts[i],tmp);
	tmp=vy;
	RBSY_SET_VELY(sim->parts[i],tmp);
	RBSY_SET_ANGL(sim->parts[i],ang);
	RBSY_SET_RVEL(sim->parts[i],rot);

	/*Phase 4 - Updating the vertices*/
	float vang,nx,ny;
	unsigned int dist;
	current_vx=RBSY_VXID(sim->parts[i]);
	while(current_vx!=RBSY_NONE)
	{
		dist=RBSY_DIST(sim->parts[current_vx]);
		vang=RBSY_ANGL(sim->parts[current_vx])+ang;
		nx=sim->parts[i].x+dist*cos(vang);
		ny=sim->parts[i].y+dist*sin(vang);
		if(nx<RBSY_BND||nx>XRES-RBSY_BND||ny<RBSY_BND||ny>YRES-RBSY_BND)
		{
			sim->kill_part(i);
			return 0;
		}
		printf("%.2f,%.2f\n",nx,ny);
		sim->parts[current_vx].x=nx;
		sim->parts[current_vx].y=ny;
		current_vx=RBSY_VXID(sim->parts[current_vx]);
	}
	/*Update phases: do until no collisions left*/
	lx=XRES,ly=YRES,mx=0,my=0;
	current_vx=RBSY_VXID(sim->parts[i]);
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
	/*Uphase 0 - Checking for collisions*/
	char cflag=0;
	float tvx=0,tvy=0;
	for(xx=lx;xx<mx;xx++)
	{
		for(yy=ly;yy<my;yy++)
		{
			if(TYP(sim->pmap[yy][xx])&&TYP(sim->pmap[yy][xx])!=PT_RBCN&&TYP(sim->pmap[yy][xx])!=PT_RBVX)
			{
				collision=min_polygon(sim,i,xx,yy);
				if(collision!=0)
				{
					cflag=1;
					forcex=-(collision>>32)&0xFFFF;
					forcey=-(collision>>16)&0xFFFF;
					if(forcex==0||forcey==0)continue;
					force=sqrt((forcex*forcex)+(forcey*forcey));
					tvx+=forcex/force;tvy+=forcey/force;
				}
			}
		}
	}
	if(!cflag)return 0;
	if(tvx==0&&tvy==0)tvy=-1;
	tvx=(tvx==0?0:(tvx<0?-1:1));
	tvy=(tvy==0?0:(tvy<0?-1:1));
	printf("Going %.2lf,%.2lf\n",tvx,tvy);
	do
	{
		/*UPhase 1 - Updating the COM*/
		sim->parts[i].x+=tvx;
		sim->parts[i].y+=tvy;

		/*UPhase 2 - Updating the vertices*/
		current_vx=RBSY_VXID(sim->parts[i]);
		while(current_vx!=RBSY_NONE)
		{
			dist=RBSY_DIST(sim->parts[current_vx]);
			vang=RBSY_ANGL(sim->parts[current_vx])+ang;
			nx=sim->parts[i].x+dist*cos(vang);
			ny=sim->parts[i].y+dist*sin(vang);
			if(nx<RBSY_BND||nx>XRES-RBSY_BND||ny<RBSY_BND||ny>YRES-RBSY_BND)
			{
				sim->kill_part(i);
				return 0;
			}
			sim->parts[current_vx].x=nx;
			sim->parts[current_vx].y=ny;
			current_vx=RBSY_VXID(sim->parts[current_vx]);
		}
		lx=XRES,ly=YRES,mx=0,my=0;
		current_vx=RBSY_VXID(sim->parts[i]);
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
		/*Uphase 0 - Checking for collisions*/
		cflag=0;
		for(xx=lx;xx<mx;xx++)
		{
			for(yy=ly;yy<my;yy++)
			{
				if(TYP(sim->pmap[yy][xx])&&TYP(sim->pmap[yy][xx])!=PT_RBCN&&TYP(sim->pmap[yy][xx])!=PT_RBVX)
				{
					if(in_polygon(sim,i,xx,yy))
					{
						cflag=1;
						goto UPhaseAgain;
					}
				}
			}
		}
		UPhaseAgain:
		continue;
	}while(cflag);
	return 0;
}

static int RBCN_GRAPHICS(GRAPHICS_FUNC_ARGS)
{
	/*We should colour all the pixels that belong to the object with our colour.*/
	/*Firstly we estabilish the bounding box*/
	unsigned int id;
	id=RBSY_CMID(*cpart);
	if(ren->sim->sys_pause)
	{
		ren->drawcircle(nx,ny,50,50,255,0,0,255);
	}
	if(RBSY_VERTICES(id,ren->sim->parts)<3)
		return 0;
	unsigned int x,y,lx=XRES,ly=YRES,mx=0,my=0,cx,cy,current_vx=RBSY_VXID(*cpart);
	while(current_vx!=RBSY_NONE)
	{
		cx=ren->sim->parts[current_vx].x;
		cy=ren->sim->parts[current_vx].y;
		lx=cx<lx?cx:lx;
		ly=cy<ly?cy:ly;
		mx=cx>mx?cx:mx;
		my=cy>my?cy:my;
		current_vx=RBSY_VXID(ren->sim->parts[current_vx]);
	}
	/*Then for every pixel that's in the polygon, we make a point in the current colour*/
	for(x=lx;x<mx;x++)
	{
		for(y=ly;y<my;y++)
		{
			if(in_polygon(ren->sim,id,x,y))
				ren->addpixel(x,y,255,0,0,255);
		}
	}
	return 0;
}