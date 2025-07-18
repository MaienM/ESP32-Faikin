// Generated case design for Faikin/Faikin.kicad_pcb
// By https://github.com/revk/PCBCase
// Generated 2025-06-21 14:09:47
// title:	PCB-FAIKIN
// rev:	1
// company:	Adrian Kennard, Andrews & Arnold Ltd
//

// Globals
margin=0.200000;
lip=2.000000;
casebottom=2.600000;
casetop=4.600000;
casewall=3.000000;
fit=0.000000;
edge=2.000000;
pcbthickness=1.200000;
nohull=false;
hullcap=1.000000;
hulledge=1.000000;
useredge=false;
spacing=51.000000;
pcbwidth=35.000000;
pcblength=16.000000;
originx=139.500000;
originy=65.000000;

module outline(h=pcbthickness,r=0){linear_extrude(height=h)offset(r=r)polygon(points=[[17.500000,-3.000000],[17.500000,3.000000],[12.500000,8.000000],[-16.750000,8.000000],[-17.125000,7.899519],[-17.399519,7.625000],[-17.500000,7.250000],[-17.500000,-7.250000],[-17.399519,-7.625000],[-17.125000,-7.899519],[-16.750000,-8.000000],[12.500000,-8.000000]],paths=[[0,1,2,3,4,5,6,7,8,9,10,11]]);}

module pcb(h=pcbthickness,r=0){linear_extrude(height=h)offset(r=r)polygon(points=[[17.500000,-3.000000],[17.500000,3.000000],[12.500000,8.000000],[-16.750000,8.000000],[-17.125000,7.899519],[-17.399519,7.625000],[-17.500000,7.250000],[-17.500000,-7.250000],[-17.399519,-7.625000],[-17.125000,-7.899519],[-16.750000,-8.000000],[12.500000,-8.000000]],paths=[[0,1,2,3,4,5,6,7,8,9,10,11]]);}
module part_R1(part=true,hole=false,block=false)
{
translate([4.000000,5.000000,1.200000])rotate([0,0,90.000000])m0(part,hole,block,casetop); // RevK:R_0402 R_0402_1005Metric (back)
};
module part_C8(part=true,hole=false,block=false)
{
translate([3.730000,-3.815000,1.200000])rotate([0,0,90.000000])m1(part,hole,block,casetop); // RevK:C_0603_ C_0603_1608Metric (back)
};
module part_V4(part=true,hole=false,block=false)
{
};
module part_D1(part=true,hole=false,block=false)
{
translate([-0.500000,3.300000,1.200000])rotate([0,0,180.000000])m2(part,hole,block,casetop); // D1 (back)
};
module part_V5(part=true,hole=false,block=false)
{
};
module part_V3(part=true,hole=false,block=false)
{
};
module part_D3(part=true,hole=false,block=false)
{
translate([4.000000,6.800000,1.200000])rotate([0,0,-90.000000])m3(part,hole,block,casetop); // D3 (back)
};
module part_C6(part=true,hole=false,block=false)
{
translate([-1.000000,6.400000,1.200000])m4(part,hole,block,casetop); // RevK:C_0402 C_0402_1005Metric (back)
};
module part_R10(part=true,hole=false,block=false)
{
translate([-0.570000,-7.000000,1.200000])rotate([0,0,180.000000])m0(part,hole,block,casetop); // RevK:R_0402 R_0402_1005Metric (back)
};
module part_D4(part=true,hole=false,block=false)
{
translate([3.150000,6.800000,1.200000])rotate([0,0,90.000000])m3(part,hole,block,casetop); // D3 (back)
};
module part_Q1(part=true,hole=false,block=false)
{
translate([1.100000,5.400000,1.200000])rotate([0,0,180.000000])m5(part,hole,block,casetop); // Q1 (back)
};
module part_U3(part=true,hole=false,block=false)
{
translate([1.230000,-5.115000,1.200000])rotate([0,0,180.000000])m6(part,hole,block,casetop); // RevK:SOT-23-6-MD8942 SOT-23-6 (back)
};
module part_R3(part=true,hole=false,block=false)
{
translate([-0.300000,7.300000,1.200000])rotate([0,0,180.000000])m0(part,hole,block,casetop); // RevK:R_0402 R_0402_1005Metric (back)
};
module part_D2(part=true,hole=false,block=false)
{
translate([3.150000,5.000000,1.200000])rotate([0,0,90.000000])m3(part,hole,block,casetop); // D3 (back)
};
module part_R11(part=true,hole=false,block=false)
{
translate([3.030000,-7.000000,1.200000])rotate([0,0,180.000000])m0(part,hole,block,casetop); // RevK:R_0402 R_0402_1005Metric (back)
};
module part_C2(part=true,hole=false,block=false)
{
translate([3.600000,2.500000,1.200000])rotate([0,0,90.000000])m4(part,hole,block,casetop); // RevK:C_0402 C_0402_1005Metric (back)
};
module part_TP1(part=true,hole=false,block=false)
{
};
module part_U4(part=true,hole=false,block=false)
{
translate([-9.670000,0.000000,1.200000])rotate([0,0,90.000000])m7(part,hole,block,casetop); // U4 (back)
};
module part_C10(part=true,hole=false,block=false)
{
translate([1.230000,-3.115000,1.200000])m1(part,hole,block,casetop); // RevK:C_0603_ C_0603_1608Metric (back)
};
module part_V2(part=true,hole=false,block=false)
{
};
module part_J1(part=true,hole=false,block=false)
{
translate([6.055000,5.000000,1.200000])rotate([0,0,-90.000000])m8(part,hole,block,casetop,05); // J1 (back)
};
module part_C5(part=true,hole=false,block=false)
{
translate([2.600000,2.800000,1.200000])rotate([0,0,90.000000])m1(part,hole,block,casetop); // RevK:C_0603_ C_0603_1608Metric (back)
};
module part_L2(part=true,hole=false,block=false)
{
translate([1.230000,-0.515000,1.200000])scale([1.000000,1.000000,1.400000])rotate([0.000000,0.000000,-90.000000])m9(part,hole,block,casetop); // RevK:L_4x4_ TYA4020 (back)
};
module part_R2(part=true,hole=false,block=false)
{
translate([1.800000,7.300000,1.200000])m0(part,hole,block,casetop); // RevK:R_0402 R_0402_1005Metric (back)
};
module part_C9(part=true,hole=false,block=false)
{
translate([-1.270000,-3.815000,1.200000])rotate([0,0,-90.000000])m1(part,hole,block,casetop); // RevK:C_0603_ C_0603_1608Metric (back)
};
module part_R9(part=true,hole=false,block=false)
{
translate([1.230000,-7.015000,1.200000])m0(part,hole,block,casetop); // RevK:R_0402 R_0402_1005Metric (back)
};
module part_PCB1(part=true,hole=false,block=false)
{
};
// Parts to go on PCB (top)
module parts_top(part=false,hole=false,block=false){
part_R1(part,hole,block);
part_C8(part,hole,block);
part_V4(part,hole,block);
part_D1(part,hole,block);
part_V5(part,hole,block);
part_V3(part,hole,block);
part_D3(part,hole,block);
part_C6(part,hole,block);
part_R10(part,hole,block);
part_D4(part,hole,block);
part_Q1(part,hole,block);
part_U3(part,hole,block);
part_R3(part,hole,block);
part_D2(part,hole,block);
part_R11(part,hole,block);
part_C2(part,hole,block);
part_TP1(part,hole,block);
part_U4(part,hole,block);
part_C10(part,hole,block);
part_V2(part,hole,block);
part_J1(part,hole,block);
part_C5(part,hole,block);
part_L2(part,hole,block);
part_R2(part,hole,block);
part_C9(part,hole,block);
part_R9(part,hole,block);
part_PCB1(part,hole,block);
}

parts_top=6;
module part_J2(part=true,hole=false,block=false)
{
};
module part_V1(part=true,hole=false,block=false)
{
};
module part_V6(part=true,hole=false,block=false)
{
};
module part_J3(part=true,hole=false,block=false)
{
};
// Parts to go on PCB (bottom)
module parts_bottom(part=false,hole=false,block=false){
part_J2(part,hole,block);
part_V1(part,hole,block);
part_V6(part,hole,block);
part_J3(part,hole,block);
}

parts_bottom=0;
module b(cx,cy,z,w,l,h){translate([cx-w/2,cy-l/2,z])cube([w,l,h]);}
module m0(part=false,hole=false,block=false,height)
{ // RevK:R_0402 R_0402_1005Metric
// 0402 Resistor
if(part)
{
	b(0,0,0,1.5,0.65,0.2); // Pad size
	b(0,0,0,1.0,0.5,0.5); // Chip
}
}

module m1(part=false,hole=false,block=false,height)
{ // RevK:C_0603_ C_0603_1608Metric
// 0603 Capacitor
if(part)
{
	b(0,0,0,1.6,0.8,1); // Chip
	b(0,0,0,1.6,0.95,0.2); // Pad size
}
}

module m2(part=false,hole=false,block=false,height)
{ // D1
// 1x1mm LED
if(part)
{
        b(0,0,0,1.2,1.2,.8);
}
if(hole)
{
        hull()
        {
                b(0,0,.8,1.2,1.2,1);
                translate([0,0,height])cylinder(d=2,h=1,$fn=16);
        }
}
if(block)
{
        hull()
        {
                b(0,0,0,2.4,2.4,1);
                translate([0,0,height])cylinder(d=4,h=1,$fn=16);
        }
}
}

module m3(part=false,hole=false,block=false,height)
{ // D3
// DFN1006-2L
if(part)
{
	b(0,0,0,1.0,0.6,0.45); // Chip
}
}

module m4(part=false,hole=false,block=false,height)
{ // RevK:C_0402 C_0402_1005Metric
// 0402 Capacitor
if(part)
{
	b(0,0,0,1.0,0.5,1); // Chip
	b(0,0,0,1.5,0.65,0.2); // Pad size
}
}

module m5(part=false,hole=false,block=false,height)
{ // Q1
if(part)
{
	b(0,0,0,1.15,2.0,1.1);
	b(0,0,0,2.1,2.0,0.6);
}
}

module m6(part=false,hole=false,block=false,height)
{ // RevK:SOT-23-6-MD8942 SOT-23-6
// SOT-23-6
if(part)
{
	b(0,0,0,1.726,3.026,1.2); // Part
	b(0,0,0,3.6,2.5,0.5); // Pins
}
}

module m7(part=false,hole=false,block=false,height)
{ // U4
// ESP32-S3-MINI-1
translate([-15.4/2,-15.45/2,0])
{
	if(part)
	{
		cube([15.4,20.5,0.8]);
		translate([0.7,0.5,0])cube([14,13.55,2.4]);
	}
	if(hole)
	{
		cube([15.4,20.5,0.8]);
	}
}
}

module m8(part=false,hole=false,block=false,height,N=0)
{ // J1
if(part)
{
	b(2.5*(N/2)-1.25,3.6,0,2.5*N+2.5,6,4);
	b(2.5*(N/2)-1.25,0,0,2.5*N+2.5,3.2,1.5);
	for(a=[0:1:N-1])translate([2.5*a,0,-3])cylinder(d1=0.5,d2=2.5,h=3,$fn=12);
}
if(hole)
{
	b(2.5*(N/2)-1.25,5+3.6,0,2.5*N+2.5,6+10,4);
}
}

module m9(part=false,hole=false,block=false,height)
{ // RevK:L_4x4_ TYA4020
// 4x4 Inductor
if(part)
{
	b(0,0,0,4,4,3);
}
}

// Generate PCB casework

height=casebottom+pcbthickness+casetop;
$fn=48;

module pyramid()
{ // A pyramid
 polyhedron(points=[[0,0,0],[-height,-height,height],[-height,height,height],[height,height,height],[height,-height,height]],faces=[[0,1,2],[0,2,3],[0,3,4],[0,4,1],[4,3,2,1]]);
}


module pcb_hulled(h=pcbthickness,r=0)
{ // PCB shape for case
	if(useredge)outline(h,r);
	else hull()outline(h,r);
}

module solid_case(d=0)
{ // The case wall
	hull()
        {
                translate([0,0,-casebottom])pcb_hulled(height,casewall-edge);
                translate([0,0,edge-casebottom])pcb_hulled(height-edge*2,casewall);
        }
}

module preview()
{
	pcb();
	color("#0f0")parts_top(part=true);
	color("#0f0")parts_bottom(part=true);
	color("#f00")parts_top(hole=true);
	color("#f00")parts_bottom(hole=true);
	color("#00f8")parts_top(block=true);
	color("#00f8")parts_bottom(block=true);
}

module top_half(step=false)
{
	difference()
	{
		translate([-casebottom-100,-casewall-100,pcbthickness-lip/2+0.01]) cube([pcbwidth+casewall*2+200,pcblength+casewall*2+200,height]);
		if(step)translate([0,0,pcbthickness-lip/2-0.01])pcb_hulled(lip,casewall/2+fit);
	}
}

module bottom_half(step=false)
{
	translate([-casebottom-100,-casewall-100,pcbthickness+lip/2-height-0.01]) cube([pcbwidth+casewall*2+200,pcblength+casewall*2+200,height]);
	if(step)translate([0,0,pcbthickness-lip/2])pcb_hulled(lip,casewall/2-fit);
}

module case_wall()
{
	difference()
	{
		solid_case();
		translate([0,0,-height])pcb_hulled(height*2);
	}
}

module top_side_hole()
{
	difference()
	{
		intersection()
		{
			parts_top(hole=true);
			case_wall();
		}
		translate([0,0,-casebottom])pcb_hulled(height,casewall-edge);
	}
}

module bottom_side_hole()
{
	difference()
	{
		intersection()
		{
			parts_bottom(hole=true);
			case_wall();
		}
		translate([0,0,edge-casebottom])pcb_hulled(height-edge*2,casewall);
	}
}

module parts_space()
{
	minkowski()
	{
		union()
		{
			parts_top(part=true,hole=true);
			parts_bottom(part=true,hole=true);
		}
		sphere(r=margin,$fn=6);
	}
}

module top_cut()
{
	difference()
	{
		top_half(true);
		if(parts_top)difference()
		{
			minkowski()
			{ // Penetrating side holes
				top_side_hole();
				rotate([180,0,0])
				pyramid();
			}
			minkowski()
			{
				top_side_hole();
				rotate([0,0,45])cylinder(r=margin,h=height,$fn=4);
			}
		}
	}
	if(parts_bottom)difference()
	{
		minkowski()
		{ // Penetrating side holes
			bottom_side_hole();
			pyramid();
		}
			minkowski()
			{
				bottom_side_hole();
				rotate([0,0,45])translate([0,0,-height])cylinder(r=margin,h=height,$fn=4);
			}
	}
}

module bottom_cut()
{
	difference()
	{
		 translate([-casebottom-50,-casewall-50,-height]) cube([pcbwidth+casewall*2+100,pcblength+casewall*2+100,height*2]);
		 top_cut();
	}
}

module top_body()
{
	difference()
	{
		intersection()
		{
			solid_case();
			pcb_hulled(height);
			top_half();
		}
		if(parts_top)minkowski()
		{
			if(nohull)parts_top(part=true);
			else hull()parts_top(part=true);
			translate([0,0,margin-height])cylinder(r=margin,h=height,$fn=8);
		}
	}
	intersection()
	{
		solid_case();
		parts_top(block=true);
	}
}

module top_edge()
{
	intersection()
	{
		case_wall();
		top_cut();
	}
}

module top_pos()
{ // Position for plotting bottom
	translate([casewall,casewall,pcbthickness+casetop])rotate([180,0,0])children();
}

module pcb_pos()
{	// Position PCB relative to base 
		translate([0,0,pcbthickness-height])children();
}

module top()
{
	top_pos()difference()
	{
		union()
		{
			top_body();
			top_edge();
		}
		parts_space();
		pcb_pos()pcb(height,r=margin);
	}
}

module bottom_body()
{ // Position for plotting top
	difference()
	{
		intersection()
		{
			solid_case();
			translate([0,0,-height])pcb_hulled(height);
			bottom_half();
		}
		if(parts_bottom)minkowski()
		{
			if(nohull)parts_bottom(part=true);
			else hull()parts_bottom(part=true);
			translate([0,0,-margin])cylinder(r=margin,h=height,$fn=8);
		}
	}
	intersection()
	{
		solid_case();
		parts_bottom(block=true);
	}
}

module bottom_edge()
{
	intersection()
	{
		case_wall();
		bottom_cut();
	}
}

module bottom_pos()
{
	translate([casewall,casewall,casebottom])children();
}

module bottom()
{
	bottom_pos()difference()
	{
		union()
		{
        		bottom_body();
        		bottom_edge();
		}
		parts_space();
		pcb(height,r=margin);
	}
}
bottom(); translate([spacing,0,0])top();
