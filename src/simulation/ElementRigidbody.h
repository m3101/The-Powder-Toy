/*
Rigidbody system common macros.
Made with <3 by Amélia O. F. da S.
Github: https://github.com/m3101
I hope you enjoy it!

I'm used to slightly-lower-level C and ASM programming, so this might not follow
the usual C++ program structure. Sorry for that...

-The Modder's Oath-

For all life will once wither,
And all life once was dead:
Proudly I come hither,
By the eternal cycle lead.

Mother of realities novel
All Built from corpses old-
When lacking in skill,
By heart I shall excel.

- ^w^

Comments will be written as if I were having a conversation with the code
(e.g. "Now we do X." or "Are we Y?"). It might be hard to get used to reading,
but it's what comes naturally to me and I accidentally noticed it too late.
Now almost everything is commented like that and I don't want to undo it all.

The Rigidbody system code was written based on what I read in
* /src/simulation/elements/STKM.cpp (figuring out graphics, or rather, how not to use graphics)
* /src/graphics/Renderer.h (actually figuring out 'how to graphics')
* /src/simulation/elements/PIPE.cpp (assuring particle.pavg is safe to mess with)
* /src/simulation/Simulation.h (Simulation properties)
* /src/simulation/ElementDefs.h (Flag ranges)

*/


/*

So... Moving solids/rigidbody physics.
Most people said it was impossible, and they did so in reasonable terms.

The basic idea I used is:
* Every solid has a centre and vertices associated to that centre.
* Objects are processed at a single particle update cycle (they act as a single particle).

A cycle happens as following:
1) At the object's centre update tick, it checks for collisions within the object
   and updates its rotational and directional speeds accordingly.
2) The centre calculates the pressure updates and its influences on its own speeds.
3) The centre updates its position and rotation according to its speeds.
4) The centre updates its vertices' positions according to its orientation and position.

To overcome the lack of storage space, I'll be using a little hack:
The "life", "tmp" and "tmp2" properties are 32bit integers. That's a lot of bits.
We probably won't be using all of them if we store only one value there. Thus, I divided them into many properties.
One can access them via bitwise math (not very intuitive if you don't know what it is) or the macros below.

MEMORY MAPPING DESCRIPTION
Areas marked with '?' are unused.

<RBCN>
*'tmp'      stores the first vertex ID and its own ID on the save (for the graphics system).
            Bit-mapping:
            [0-1-2-3-4-5-6-7-8-9-A-B-C-D-E-F-0-1-2-3-4-5-6-7-8-9-A-B-C-D-E-F]
            [UINT16     RBSY_VXID           |UINT16     RBSY_CMID           ]
                - This gives us support for IDs up to 65534 (0xFF will be our "NULL"). Yeah. I know. This might be a problem, but I'm doing this for fun.
*'tmp2'     stores the rotational velocity
            Bit-mapping:
            [0-1-2-3-4-5-6-7-8-9-A-B-C-D-E-F-0-1-2-3-4-5-6-7-8-9-A-B-C-D-E-F]
            [FLT32                      RBSY_RVEL                           ]
*'pavg[0]'  stores the particle X velocity component
            Bit-mapping:
            [0-1-2-3-4-5-6-7-8-9-A-B-C-D-E-F-0-1-2-3-4-5-6-7-8-9-A-B-C-D-E-F]
            [FLT32                      RBSY_VELX                           ]
*'pavg[1]'  stores the particle Y velocity component
            Bit-mapping:
            [0-1-2-3-4-5-6-7-8-9-A-B-C-D-E-F-0-1-2-3-4-5-6-7-8-9-A-B-C-D-E-F]
            [FLT32                      RBSY_VELY                           ]
*'life'     stores the rigidbody centre "orientation" information
            Bit-mapping:
            [0-1-2-3-4-5-6-7-8-9-A-B-C-D-E-F-0-1-2-3-4-5-6-7-8-9-A-B-C-D-E-F]
            [FLT32                      RBSY_ANGL                           ]
<RBVX>
*'tmp'      stores the next vertex ID and this vertex's distance from the centre.
            Bit-mapping:
            [0-1-2-3-4-5-6-7-8-9-A-B-C-D-E-F-0-1-2-3-4-5-6-7-8-9-A-B-C-D-E-F]
            [UINT16     RBSY_VXID           |UINT16       RBSY_DIST         ]
                - This gives us support for IDs up to 65534 (0xFF will be our "NULL"). Yeah. I know. This might be a problem, but I'm doing this for fun.
*'tmp2'     stores the rotational inertia of the whole object (for the first vertex only)
            Bit-mapping:
            [0-1-2-3-4-5-6-7-8-9-A-B-C-D-E-F-0-1-2-3-4-5-6-7-8-9-A-B-C-D-E-F]
            [UINT16    RBSY_ROIN            |???????????????????????????????]
*'life'     stores the angle this vertex makes with the horizon at "0" orientation
            Bit-mapping:
            [0-1-2-3-4-5-6-7-8-9-A-B-C-D-E-F-0-1-2-3-4-5-6-7-8-9-A-B-C-D-E-F]
            [FLT32                      RBSY_ANGL                           ]
*/

/*Macros for accessing the properties*/
#define RBSY_VXID(part)             ((uint16_t )(((part).tmp&0xFFFF0000)>>16)&0xFFFF)
#define RBSY_SET_VXID(part,id)      (part).tmp&=0x0000FFFF;(part).tmp|=((uint32_t)id&0xFFFF)<<16;

#define RBSY_CMID(part)             ((uint16_t )((part).tmp&0xFFFF))
#define RBSY_SET_CMID(part,id)      (part).tmp&=0xFFFF0000;(part).tmp|=((uint32_t)id&0xFFFF);

#define RBSY_ROIN(part)             ((uint16_t )(((part).tmp2&0xFFFF0000)>>16)&0xFFFF)
#define RBSY_SET_ROIN(part,r)      (part).tmp2&=0x0000FFFF;(part).tmp2|=((uint32_t)r&0xFFFF)<<16;

#define RBSY_DIST(part)             ((uint16_t )((part).tmp&0x0000FFFF))
#define RBSY_SET_DIST(part,d)      (part).tmp&=0xFFFF0000;(part).tmp|=((uint32_t)d&0xFFFF);

#define RBSY_RVEL(part)             (*((float*)&(part).tmp2))
#define RBSY_SET_RVEL(part,rvel)    *((float*)&(part).tmp2)=rvel;

#define RBSY_VELX(part)             (*((float*)&(part).pavg[0]))
#define RBSY_SET_VELX(part,v)      *((float*)&(part).pavg[0])=v;

#define RBSY_VELY(part)             (*((float*)&(part).pavg[1]))
#define RBSY_SET_VELY(part,v)      *((float*)&(part).pavg[1])=v;

#define RBSY_ANGL(part)             (*((float*)&(part).life))
#define RBSY_SET_ANGL(part,angl)    *((float*)&(part).life)=angl;

/*A macro for finding the last vertex of a RBCN particle*/
#define RBSY_LAST_VERTEX(centre_id,parts) ({unsigned int __i___=centre_id;while(RBSY_VXID(parts[__i___])!=RBSY_NONE)__i___=RBSY_VXID(parts[__i___]);__i___;})

/*A macro that returns the length of a vertex list*/
#define RBSY_VERTICES(centre_id,parts) ({unsigned int __i___=RBSY_VXID(parts[centre_id]),l=0;while(__i___!=RBSY_NONE){l++;__i___=RBSY_VXID(parts[__i___]);};l;})

/*Constants*/
#define RBSY_TAU 1.57079f
#define RBSY_PI  3.14159f
#define RBSY_2PI 6.28318f

/*World boundary margin (apparently going beyond that makes the particle disappear)*/
#define RBSY_BND 5

/*"Null pointer" ID for the ID fields*/
#define RBSY_NONE 0xFFFF