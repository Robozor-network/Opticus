prumer = 100;
rafek = 10;
tloustka_rafku = 10;
tloustka_zebrovani = 5;
tloustka_stredu = 12;
prumer_stredu = 20;
prumer_hridele = 4.9;
polomer_zakriveni = 15;
dokonalost_kruhu = 100;
vyska_loziska = 3;
mount_hole = 3.2;       // mounting screw hole diameter

pocet_vykrojeni = 6;
vnitrni_polomer_zebrovani = 10;

module kolo(){
	intersection() {
		difference() {
			cylinder(r=prumer/2, h=tloustka_rafku, center=true, $fn=dokonalost_kruhu);
			cylinder(r=prumer/2-rafek, h=tloustka_rafku*2, center=true, $fn=dokonalost_kruhu);
		}
		rotate_extrude(convexity=10, $fn=dokonalost_kruhu){translate([prumer/2-polomer_zakriveni, 0, 0]){circle(r = polomer_zakriveni, $fn=dokonalost_kruhu);}}
	}

	difference() {
		union() {
			translate([0,0,-(tloustka_rafku-tloustka_zebrovani)/2]) cylinder(r=prumer/2-tloustka_rafku, h=tloustka_zebrovani, center=true, $fn=dokonalost_kruhu);
			translate([0,0,-(tloustka_rafku- tloustka_stredu)/2]) cylinder(r=prumer_stredu/2, h=tloustka_stredu, center=true, $fn=dokonalost_kruhu);
		}
        
        translate([0,0,-tloustka_stredu/2 + vyska_loziska/2 ])
            cylinder(r=4, h=vyska_loziska ,center=true, $fn=20);	// vybrani na mozazne kluzne lozisko na vystupu motoru

        rotate([0,0,30]){            
            translate([0,30,2])
                rotate([90,0,0])
                    cylinder (h = 30, r= mount_hole/2, $fn=20); // hole for mounting screw. 
            
            translate([-3 ,3, -2])       // hole for mounting nut. 
                cube([6, 3, 20], center = false);

            difference() {          // D-cut on axis
                translate([0,0,  tloustka_stredu-vyska_loziska + 0.6])
                    cylinder(r=prumer_hridele/2, h=tloustka_stredu*2, center=true, $fn=20);
                translate([-prumer_hridele/2,prumer_hridele/2 - 0.5,-2])
                    cube([10, 10, 200], center=false);
            }
        }
	}
}


module vykrojeni(alfa,vnitrni_polomer,r,tloustka,sirka_okraje,polomer_rohu){
	b = sin(alfa/2) * r;

	intersection() {
		translate([r/2 + sirka_okraje/sin(alfa/2),0,0]){
			difference(){
				cube([r, 2*b, tloustka], center=true);
				union(){
					translate([-r/2,0,-tloustka]){
						rotate([0,0,alfa/2]){
							cube([r, b, tloustka*2], center=false);
						}
					}
					translate([-r/2 - sin(alfa/2) * b,-cos(alfa/2) * b,-tloustka]){
						rotate([0,0,-alfa/2]){
							cube([r, b, tloustka*2], center=false);
						}
					}
				}
			}
		}

		difference() {
			cylinder(r=r, h=tloustka, center=true,$fn=dokonalost_kruhu);
			cylinder(r=vnitrni_polomer, h=tloustka*2, center=true,$fn=dokonalost_kruhu);
		}
	}
}

difference() {
	kolo();
	difference() {
		for (i=[0:pocet_vykrojeni-1]){
				rotate([0,0,i * 360/pocet_vykrojeni]){
					scale([1,1,3]) {
						vykrojeni(360/pocet_vykrojeni,10,37,10,3);
					}
				}
			}

		translate([0,0,-tloustka_zebrovani*2]) {cylinder(r=vnitrni_polomer_zebrovani, h=tloustka_zebrovani*3, center=false, $fn=dokonalost_kruhu);}
	}

}
