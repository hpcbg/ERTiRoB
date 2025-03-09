module base_base() {
    w = 3;
    translate([-116.5,-65.7,0]) cylinder(75,11,11,true);
    translate([116.5,-65.7,0]) cylinder(75,11,11,true);
    translate([-116.5,65.7,0]) cylinder(75,11,11,true);
    translate([116.5,65.7,0]) cylinder(75,11,11,true);
    translate([0,-65.7,0]) cylinder(75,11,11,true);
    translate([0,65.7,0]) cylinder(75,11,11,true);
    
    translate([0,0,-37.5+w/2]) cube([116.5*2,65.7*2,w],true);
    
    translate([-116.5-11/2,0,-37.5+w/2]) cube([11,65.7*2,w],true);
    translate([+116.5+11/2,0,-37.5+w/2]) cube([11,65.7*2,w],true);
    translate([0,-65.7-11/2,-37.5+w/2]) cube([116.5*2,11,w],true);
    translate([0,+65.7+11/2,-37.5+w/2]) cube([116.5*2,11,w],true);
    
    translate([-116.5-11/2,0,-37.5+w/2]) cube([11,65.7*2,w],true);
    translate([+116.5+11/2,0,-37.5+w/2]) cube([11,65.7*2,w],true);
    translate([0,-65.7-11/2,-37.5+w/2]) cube([116.5*2,11,w],true);

    translate([0,+65.7+11-w/2,0]) cube([116.5*2,w,75],true);
    translate([0,-65.7-11+w/2,0]) cube([116.5*2,w,75],true);
    translate([-116.5-11+w/2,0,0]) cube([w,65.7*2,75],true);
    translate([+116.5+11-w/2,0,0]) cube([w,65.7*2,75],true);
}

module base() {
    difference() {
        base_base();
        union() {
            translate([0,0,55]) screw_holes();
            translate([0,0,3]) cube([249,122,75], true);
            cube([220,110,73], true);
            translate([116.5/2,0,3]) cube([105,147.4,75], true);
            translate([-116.5/2,0,3]) cube([105,147.4,75], true);
        }
    }
}

module base1() {
    difference() {
        base();
        union() {
            rotate([0,0,-30]) translate([-215,-200,-100]) cube([200,400,200]);
            translate([-56,-65.7,-36]) cylinder(3,8,8,true);
            translate([-48,-65.7,-36]) cylinder(3,8,8,true);
            translate([21,65.7,-36]) cylinder(3,8,8,true);
            translate([29,65.7,-36]) cylinder(3,8,8,true);
            translate([28,65.7+8.25,0]) cube([10,0.5,69],true);
            translate([-58,-65.7-8.25,0]) cube([10,0.5,69],true);
            base_glue_piece();
        }
    }
}

module base2() {
    difference() {
        base();
        union() {
            rotate([0,0,-30]) translate([-15,-200,-100]) cube([200,400,200]);
            base_glue_piece();
        }
    }
    translate([-56,-65.7,-36]) cylinder(3,7.9,7.9,true);
    translate([-48,-65.7,-36]) cylinder(3,7.9,7.9,true);
    translate([21,65.7,-36]) cylinder(3,7.9,7.9,true);
    translate([29,65.7,-36]) cylinder(3,7.9,7.9,true);
    translate([28,65.7+8.25,0]) cube([10,0.5,68.8],true);
    translate([-58,-65.7-8.25,0]) cube([10,0.5,68.8],true);
}

module base_glue_piece() {
    difference() {
        translate([0,0,-36.75]) cube([200,110,0.5], true);
        union() {
            rotate([0,0,-30]) translate([-5,-200,-100]) cube([200,400,200]);
            rotate([0,0,-30]) translate([-225,-200,-100]) cube([200,400,200]);
        }
    }
}

module cover_base() {
    coverH = 4;
    translate([-116.5,-65.7,0]) cylinder(coverH,11,11,true);
    translate([116.5,-65.7,0]) cylinder(coverH,11,11,true);
    translate([-116.5,65.7,0]) cylinder(coverH,11,11,true);
    translate([116.5,65.7,0]) cylinder(coverH,11,11,true);
    translate([0,-65.7,0]) cylinder(coverH,11,11,true);
    translate([0,65.7,0]) cylinder(coverH,11,11,true);
    translate([-116.5-11/2,0,0]) cube([11,65.7*2,coverH],true);
    translate([+116.5+11/2,0,0]) cube([11,65.7*2,coverH],true);
    translate([0,-73.7,0]) cube([116.5*2,6,coverH],true);
    translate([0,+72.7,0]) cube([116.5*2,8,coverH],true);
        
    translate([13,6,0])
    linear_extrude(height = coverH, center = true, convexity = 10)
    import (file = "resources/lid.dxf"); // The file is taken from https://github.com/peterso/robotlearningblock/blob/main/assets/manufacturing/TBv2023
}

module screw_holes() {
    translate([-116.5,-65.7,0]) cylinder(75,1,1,true);
    translate([116.5,-65.7,0]) cylinder(75,1.5,1.5,true);
    translate([-116.5,65.7,0]) cylinder(75,1.5,1.5,true);
    translate([116.5,65.7,0]) cylinder(75,1.5,1.5,true);
    translate([0,-65.7,0]) cylinder(75,1.5,1.5,true);
    translate([0,65.7,0]) cylinder(75,1.5,1.5,true);
}

module cover() {
    difference() {
        rotate([180,0,180]) cover_base();
        screw_holes();
    }
}

module cover1() {
    difference() {
        cover();
        rotate([0,0,20]) translate([-215,-200,-5]) cube([200,400,10]);
    }
}

module cover2() {
    difference() {
        cover();
        rotate([0,0,20]) translate([-15,-200,-5]) cube([200,400,10]);
    }
}

module door() {
    rotate([0,180,180])
   linear_extrude(height = 3, center = true, convexity = 10)
   import (file = "resources/door.dxf");  // The file is taken from https://github.com/peterso/robotlearningblock/blob/main/assets/manufacturing/TBv2023
}

module hinge_door() {
    difference() {
        union() {
            translate([0,-1,0]) cube([75,34,3], true);
            translate([30,-18,3.5]) rotate([0,90,0]) cylinder(15,5,5, true);
            translate([-30,-18,3.5]) rotate([0,90,0]) cylinder(15,5,5, true);
            translate([0,-18,3.5]) rotate([0,90,0]) cylinder(15,5,5, true);
        }
        union() {
            translate([15,-18,3.5]) rotate([0,90,0]) cube([10.01,10.01,15], true);
            translate([-15,-18,3.5]) rotate([0,90,0]) cube([10.01,10.01,15], true);
            translate([0,-18,3.5]) rotate([0,90,0]) cylinder(200,0.5,0.5, true);
            translate([-45,-18,3.5]) rotate([0,90,0]) cylinder(50,2.85,2.85, true);
            translate([27.5,7,0]) cylinder(20,2,2,true);
            translate([-27.5,7,0]) cylinder(20,2,2,true);
        }
    }
}

module hinge_cover() {
    difference() {
        union() {
            translate([0,-1,0]) cube([75,34,3], true);
            translate([15,-18,3.5]) rotate([0,90,0]) cylinder(14.5,5,5, true);
            translate([-15,-18,3.5]) rotate([0,90,0]) cylinder(14.5,5,5, true);
        }
        union() {
            translate([30,-18,3.5]) rotate([0,90,0]) cube([10.01,10.01,15.49], true);
            translate([-30,-18,3.5]) rotate([0,90,0]) cube([10.01,10.01,15.49], true);
            translate([0,-18,3.5]) rotate([0,90,0]) cube([10.01,10.01,15.49], true);
            translate([0,-18,3.5]) rotate([0,90,0]) cylinder(200,0.5,0.5, true);
            translate([27.5,7,0]) cylinder(20,2,2,true);
            translate([-27.5,7,0]) cylinder(20,2,2,true);
        }
    }
}

module terminal_block_bracket() {
    translate([-50,-147/2,-75/2+3]) cube([20,147,10]);
    difference() {
        translate([-50,-50/2,-75/2+3]) cube([20,50,50]);
        translate([-40,0,15]) cube([8,40,45],true);
    }
}

module probe_holder() {
    import("resources/Probe Holder - Probe Holder.stl"); // The file is taken from https://github.com/peterso/robotlearningblock/blob/main/assets/manufacturing/TBv2023
    translate([-67,0,0]) cube([19.7, 20, 4]);
    difference() {
        translate([-67,0,0]) cube([5, 20, 20]);
        translate([-67,10,20]) rotate([0,90,0]) cylinder(20,5,5, true);
    }
    translate([-64.5,17.5,20]) rotate([0,90,0]) cylinder(5,2.5,2.5, true);
    translate([-64.5,2.5,20]) rotate([0,90,0]) cylinder(5,2.5,2.5, true);
    difference() {
        translate([-38,2,0]) cube([5, 16, 28]);
        translate([-36,10,19]) rotate([0,90,0]) cylinder(22,5,5, true);
    }
}

module probe_holder_without_stand() {
    import("resources/Probe Holder - Probe Holder.stl"); // The file is taken from https://github.com/peterso/robotlearningblock/blob/main/assets/manufacturing/TBv2023
    difference() {
        translate([-38,2,0]) cube([5, 16, 28]);
        translate([-36,10,19]) rotate([0,90,0]) cylinder(22,5,5, true);
    }
}

module hinge_adapter() { 
    difference() {
        import("resources/Angle Sensor-Hinge Adapter.stl"); // The file is taken from https://github.com/peterso/robotlearningblock/blob/main/assets/manufacturing/TBv2023
        translate([0,8.5,18.4]) cylinder(25,10,10);
    }
    translate([0,8.5,18]) cylinder(13,2.6,2.6);
}

module angle_sensor_bracket() {
    import("resources/Angle Sensor Mounting Bracket - Angle Sensor Mounting Bracket.stl"); // The file is taken from https://github.com/peterso/robotlearningblock/blob/main/assets/manufacturing/TBv2023
}

module left_wrap_post() {
    rotate([-35,0,180]) import("resources/Cable_Winding - Cable Wrap Post Left.stl"); // The file is taken from https://github.com/peterso/robotlearningblock/blob/main/assets/manufacturing/TBv2023
}

module right_wrap_post() {
    rotate([-35,0,180]) import("resources/Cable_Winding - Cable Wrap Post Right.stl"); // The file is taken from https://github.com/peterso/robotlearningblock/blob/main/assets/manufacturing/TBv2023
}

module assembled() {
    translate([0,0,37.5]) color("red") base();
    translate([0,0,37.5]) terminal_block_bracket();
    color("blue") translate([0,0,77]) cover();
    translate([-89,58,84]) {
        color("green") translate([0,0,-0.5]) door();
        translate([42.5,-70,-3.5]) rotate([180,0,180]) hinge_door();
        translate([42.5,-106,-10.5]) rotate([0,0,180]) hinge_cover();
    }
    translate([-111.4,-73,79]) rotate([0,0,-90]) probe_holder();
    translate([16,-38.4,77]) rotate([0,-90,0]) hinge_adapter();
    translate([13,-10,54.5]) angle_sensor_bracket();
    translate([16,90,85.9]) left_wrap_post();
    translate([-50,90,85.9]) right_wrap_post();
}

$fn=180;
// base();
// base1();
// base2();
// base_glue_piece();
// cover();
// cover1();
// cover2();
// door();
// hinge_door();
// hinge_cover();
// hinge_adapter();
// terminal_block_bracket();
// probe_holder();
// probe_holder_without_stand();
// angle_sensor_bracket();
// left_wrap_post();
// right_wrap_post();

assembled();