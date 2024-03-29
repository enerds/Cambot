module flanschplatte()
{
	/* radius für spheres etc. von durchmesser */
	function r_from_dia(d) = d / 2;

	/* cylinder erstellen und direkt rotieren */
	module rotcy(rot, r, h) {
		rotate(90, rot)
			cylinder(r = r, h = h/2, center = true, $fn=40);
	}

	difference() {
		/* platte an sich */
		translate([0, -block_y, 0]) cube([block_x, 2, block_z], center=true);

		/* querschrauben für flansch-halterungen */
		translate([-block_x/2 + 3*schrauben_durchmesser, -block_y, -block_z/5]) rotcy([1, 0, 0], r_from_dia(schrauben_durchmesser), block_y+0.2);
		translate([block_x/2 - 3*schrauben_durchmesser,  -block_y, -block_z/5]) rotcy([1, 0, 0], r_from_dia(schrauben_durchmesser), block_y+0.2);
		translate([-block_x/2 + 3*schrauben_durchmesser, -block_y, block_z/5]) rotcy([1, 0, 0], r_from_dia(schrauben_durchmesser), block_y+0.2);
		translate([block_x/2 - 3*schrauben_durchmesser,  -block_y, block_z/5]) rotcy([1, 0, 0], r_from_dia(schrauben_durchmesser), block_y+0.2);


		/* aussparungen für schraublöcher */
		translate([-block_x/2-0.2,-block_y,0]) cube([block_x/3, block_y/2, block_z+0.5], center=true);
		translate([block_x/2+0.2,-block_y,0]) cube([block_x/3, block_y/2, block_z+0.5], center=true);

		/* aussparung für kugellager */
		translate([0, -block_y, 0]) rotcy([1, 0, 0], r_from_dia(kugellager_durchmesser), 5);

		/* extra aussparung für den flansch */
		translate([0, -block_y+1, 0]) rotcy([1, 0, 0], r_from_dia(kugellager_durchmesser + 2*flansch_breite), 0.5);	

		/* material weiter reduzieren */
		translate([0, -block_y, -block_z/2]) cube([block_x+0.5, block_y+0.5, block_z/3], center=true); 
	}
}

module lagerblock()
{
	/* radius für spheres etc. von durchmesser */
	function r_from_dia(d) = d / 2;

	/* cylinder erstellen und direkt rotieren */
	module rotcy(rot, r, h) {
		rotate(90, rot)
			cylinder(r = r, h = h, center = true, $fn=40);
	}

	difference() {
		/* block an sich */
		cube([block_x, block_y, block_z], center=true);

		/* schraublöcher */
		translate([-block_x/2 + schrauben_durchmesser, -block_y/3, 0]) rotcy([0, 0, 0], cy_r, block_x+0.2);
		translate([-block_x/2 + schrauben_durchmesser, block_y/3,0]) rotcy([0, 0, 0], cy_r, block_x+0.2);
		translate([block_x/2 - schrauben_durchmesser, -block_y/3,0]) rotcy([0, 0, 0], cy_r, block_x+0.2);
		translate([block_x/2 - schrauben_durchmesser, block_y/3,0]) rotcy([0, 0, 0], cy_r, block_x+0.2);

		/* aussparungen für schraublöcher */
		translate([-block_x/2-0.2,  block_y/3, -3]) cube([block_x/2-schrauben_durchmesser, block_y/2, block_z], center=true);
		translate([block_x/2+0.2,  block_y/3, -3]) cube([block_x/2-schrauben_durchmesser, block_y/2, block_z], center=true);
		translate([block_x/2+0.2, -block_y/3, -3]) cube([block_x/2-schrauben_durchmesser, block_y/2, block_z], center=true);
		translate([-block_x/2-0.2, -block_y/3, -3]) cube([block_x/2-schrauben_durchmesser, block_y/2, block_z], center=true);

		/*
		translate([-block_x/2 + schrauben_durchmesser, -block_y/3, -3]) rotcy([0, 0, 0], cy_r+2, block_z);
		translate([-block_x/2 + schrauben_durchmesser, block_y/3,-3]) rotcy([0, 0, 0], cy_r+2, block_z);
		translate([block_x/2 - schrauben_durchmesser, -block_y/3,-3]) rotcy([0, 0, 0], cy_r+2, block_z);
		translate([block_x/2 - schrauben_durchmesser, block_y/3,-3]) rotcy([0, 0, 0], cy_r+2, block_z);
		*/

		/* querschrauben für flansch-halterungen */
		translate([-block_x/2 + 3*schrauben_durchmesser, 0, -block_z/5]) rotcy([1, 0, 0], r_from_dia(schrauben_durchmesser), block_y+0.2);
		translate([block_x/2 - 3*schrauben_durchmesser,  0, -block_z/5]) rotcy([1, 0, 0], r_from_dia(schrauben_durchmesser), block_y+0.2);
		translate([-block_x/2 + 3*schrauben_durchmesser, 0, block_z/5]) rotcy([1, 0, 0], r_from_dia(schrauben_durchmesser), block_y+0.2);
		translate([block_x/2 - 3*schrauben_durchmesser,  0, block_z/5]) rotcy([1, 0, 0], r_from_dia(schrauben_durchmesser), block_y+0.2);

		/* material weiter reduzieren */
		translate([0, 0, -block_z/2]) cube([block_x+0.5, block_y+0.5, block_z/3], center=true); 
		translate([-block_x/2-0.2,0,-5]) cube([block_x/2-schrauben_durchmesser, block_y, block_z - 5], center=true);
		translate([block_x/2+0.2,0,-5]) cube([block_x/2-schrauben_durchmesser, block_y, block_z - 5], center=true);

		/* achsdurchführung */
		rotcy([1, 0, 0], r_from_dia(wellen_durchmesser), cy_h);
	
		/* kugellager-aussparungen */
		translate([0, block_y/2 - kugellager_tiefe/2, 0]) rotcy([1, 0, 0], r_from_dia(kugellager_durchmesser), kugellager_tiefe+0.1);
		translate([0, -block_y/2 + kugellager_tiefe/2, 0]) rotcy([1, 0, 0], r_from_dia(kugellager_durchmesser), kugellager_tiefe+0.1);
	
	}


	size = 50;

	cy_r = r_from_dia(schrauben_durchmesser);
}


block_x = 40; // 5cm
block_y = 20; // 2.5cm
block_z = 32; // 2.5cm

cy_h = block_x/2+0.4;

wellen_durchmesser = 6; // eigentlich 4, aber mehr tut nicht weh
schrauben_durchmesser = 4;
kugellager_durchmesser = 8;
kugellager_tiefe = 3;
flansch_breite = .5;

flanschplatte();
rotate(180, 1) flanschplatte();
lagerblock(); 