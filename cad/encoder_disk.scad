// Encoder disk for the wheel odometer.
// Measurements taken off the original disk: 12 evenly spaced blades,
// 24.30 mm across the blade tips, 2.50 mm shaft bore.

$fn = 180;

blades       = 12;
d_total      = 24.30;  // across blade tips
d_shaft      = 2.15;
blade_length = 6.00;
blade_tip_w  = 3.00;   // width at the outer end
blade_root_w = 1.80;   // width where it meets the hub
thickness    = 0.50;

d_cap        = 4.90;   // cap over the shaft, protrudes from one face
h_cap        = 5.20;   // measured above the face, so 8.70 mm overall

d_mouth      = 3.00;   // funnel that guides the axle into the bore
h_funnel     = thickness;

r_tip = d_total / 2;
r_hub = r_tip - blade_length;

overlap = 0.01;  // sinks the blade root into the hub so the union is clean

module blade_2d() {
    polygon([
        [r_hub - overlap, -blade_root_w / 2],
        [r_tip,           -blade_tip_w  / 2],
        [r_tip,            blade_tip_w  / 2],
        [r_hub - overlap,  blade_root_w / 2],
    ]);
}

module disk_2d() {
    union() {
        circle(r = r_hub);
        for (i = [0 : blades - 1])
            rotate(i * 360 / blades) blade_2d();
    }
}

module encoder_disk() {
    difference() {
        union() {
            linear_extrude(height = thickness) disk_2d();
            cylinder(h = thickness + h_cap, d = d_cap);
        }
        // Shaft bore, through the disk and the cap
        translate([0, 0, -1])
            cylinder(h = thickness + h_cap + 2, d = d_shaft);

        // Funnel on the face opposite the cap. The straight section sits
        // below z=0, outside the part, so the mouth is exactly d_mouth.
        cylinder(h = h_funnel, d1 = d_mouth, d2 = d_shaft);
        translate([0, 0, -1]) cylinder(h = 1, d = d_mouth);
    }
}

encoder_disk();

// Blade pitch at the root is 2*PI*r_hub/blades = 3.22 mm, so the slots
// between roots are 1.42 mm wide -- check this against your nozzle/kerf.
