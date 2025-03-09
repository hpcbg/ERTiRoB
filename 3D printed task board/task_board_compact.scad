base_h = 50;

module base_base() {
    w = 2.5;
    h = base_h;
    translate([-116.5,-65.7,0]) cylinder(h,11,11,true);
    translate([116.5,-65.7,0]) cylinder(h,11,11,true);
    translate([-116.5,65.7,0]) cylinder(h,11,11,true);
    translate([116.5,65.7,0]) cylinder(h,11,11,true);
    translate([0,-65.7,0]) cylinder(h,11,11,true);
    translate([0,65.7,0]) cylinder(h,11,11,true);
    
    translate([0,0,-h/2+w/2]) cube([116.5*2,65.7*2,w],true);
    
    translate([-116.5-11/2,0,-h/2+w/2]) cube([11,65.7*2,w],true);
    translate([+116.5+11/2,0,-h/2+w/2]) cube([11,65.7*2,w],true);
    translate([0,-65.7-11/2,-h/2+w/2]) cube([116.5*2,11,w],true);
    translate([0,+65.7+11/2,-h/2+w/2]) cube([116.5*2,11,w],true);
    
    translate([-116.5-11/2,0,-h/2+w/2]) cube([11,65.7*2,w],true);
    translate([+116.5+11/2,0,-h/2+w/2]) cube([11,65.7*2,w],true);
    translate([0,-65.7-11/2,-h/2+w/2]) cube([116.5*2,11,w],true);

    translate([0,+65.7+11-w/2,0]) cube([116.5*2,w,h],true);
    translate([0,-65.7-11+w/2,0]) cube([116.5*2,w,h],true);
    translate([-116.5-11+w/2,0,0]) cube([w,65.7*2,h],true);
    translate([+116.5+11-w/2,0,0]) cube([w,65.7*2,h],true);
}

module screw_holes() {
    translate([-116.5,-65.7,0]) cylinder(75,1,1.5,true);
    translate([116.5,-65.7,0]) cylinder(75,1.5,1.5,true);
    translate([-116.5,65.7,0]) cylinder(75,1.5,1.5,true);
    translate([116.5,65.7,0]) cylinder(75,1.5,1.5,true);
    translate([0,-65.7,0]) cylinder(75,1.5,1.5,true);
    translate([0,65.7,0]) cylinder(75,1.5,1.5,true);
}

module mount_point() {
    difference() {
        cube([10,10,10], true);
        translate([0,0,5]) cylinder(3,1.5,1.5,true);
    }
}

module base() {
    difference() {
        base_base();
        union() {
            translate([0,0,30]) screw_holes();
            translate([0,0,2.5]) cube([250,120,base_h], true);
            cube([221.5,120,base_h-3], true);
            translate([116.5/2,0,2.5]) cube([105,148.4,base_h], true);
            translate([-116.5/2,0,2.5]) cube([105,148.4,base_h], true);
            translate([0,0,base_h/2-3.1/2]) cover_blank(3.11,8.5);
        }
    }
    translate([-45,-65.7-3.5,-base_h/2+5]) mount_point();
    translate([-45,+65.7+3.5,-base_h/2+5]) mount_point();
    translate([45,-65.7-3.5,-base_h/2+5]) mount_point();
    translate([45,+65.7+3.5,-base_h/2+5]) mount_point();
    translate([90,-65.7-3.5,-base_h/2+5]) mount_point();
    translate([90,+65.7+3.5,-base_h/2+5]) mount_point();
    translate([-90,-65.7-3.5,-base_h/2+5]) mount_point();
    translate([-90,+65.7+3.5,-base_h/2+5]) mount_point();
    
    
}

module base_glue_piece() {
    cube([30,120,0.5], true);
}

module base_half() {
    difference() {
        base();
        union() {
            rotate([0,0,10]) translate([0,-200,-100]) cube([200,400,200]);
            translate([0,0,-base_h/2+1.25]) base_glue_piece();
            translate([10.6,-70,-base_h/2+1.25]) cylinder(2.5,5,5,true);
            translate([13,-65.7-8.75,0]) cube([10,0.5,44],true);
        }
    }
    translate([-10.6,70,-base_h/2+1.25]) cylinder(2.5,4.8,4.8,true);
    translate([-13,65.7+8.75,0]) cube([9,0.5,43],true);

}

module cover_blank(coverH = 3, coverR = 8.3) {
    translate([-116.5,-65.7,0]) cylinder(coverH,coverR,coverR,true);
    translate([116.5,-65.7,0]) cylinder(coverH,coverR,coverR,true);
    translate([-116.5,65.7,0]) cylinder(coverH,coverR,coverR,true);
    translate([116.5,65.7,0]) cylinder(coverH,coverR,coverR,true);
    translate([0,-65.7,0]) cylinder(coverH,coverR,coverR,true);
    translate([0,65.7,0]) cylinder(coverH,coverR,coverR,true);
    translate([-116.5-coverR/2,0,0]) cube([coverR,65.7*2,coverH],true);
    translate([+116.5+coverR/2,0,0]) cube([coverR,65.7*2,coverH],true);
    translate([0,-65.7-coverR/2,0]) cube([116.5*2,coverR,coverH],true);
    translate([0,+65.7+coverR/2,0]) cube([116.5*2,coverR,coverH],true);
    cube([235,140,coverH], true);
}

module cover_base() {
    coverH = 3;
    difference() {
        cover_blank();
        union() {
            cube([220,120,10],true);
            translate([-60,40,0]) cube([80,60,10], true);
            translate([60,40,0]) cube([80,60,10], true);
            translate([70,-40,0]) cube([80,61,10], true);
        }
    }
    translate([13,6,0]) linear_extrude(height = coverH, center = true, convexity = 10) import (file = "resources/lid.dxf"); // The file is taken from https://github.com/peterso/robotlearningblock/blob/main/assets/manufacturing/TBv2023
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
        union() {
            translate([-190,-118,-5]) cube([200,100,10]);
            translate([-212,-20,-5]) cube([200,100,10]);
        }
    }
}

module cover2() {
    difference() {
        cover();
        union() {
            translate([10,-118,-5]) cube([200,100,10]);
            translate([-12,-20,-5]) cube([200,100,10]);
        }
    }
}

module door() {
   rotate([0,180,180]) linear_extrude(height = 3, center = true, convexity = 10) import (file = "resources/door.dxf"); // The file is taken from https://github.com/peterso/robotlearningblock/blob/main/assets/manufacturing/TBv2023
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
    difference() {
        translate([-55,-147/2,3]) cube([20,147,10]);
        union() {
            translate([-45,-65.7-3.5,3]) mount_point();
            translate([-45,+65.7+3.5,3]) mount_point();
            translate([-45,-65.7-3.5,0]) cylinder(100,1.5,1.5,true);
            translate([-45,+65.7+3.5,0]) cylinder(100,1.5,1.5,true);
        }
    }
    difference() {
        translate([-55,-50/2,8]) cube([20,50,20]);
        translate([-45,0,15+12.5]) cube([8,40,45],true);
    }
    //translate([-45,0,base_h - 45/2]) color("red") cube([8,40,45],true);
}

module probe_holder() {
    import("resources/Probe Holder - Probe Holder.stl");
    // The file is taken from https://github.com/peterso/robotlearningblock/blob/main/assets/manufacturing/TBv2023
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
    translate([0,0,50/2]) color("red") base();
    color("blue") translate([0,0,base_h-1.5]) cover();
    translate([-89,58,base_h + 6]) {
        color("green") translate([0,0,-0.5]) door();
        translate([42.5,-70,-3.5]) rotate([180,0,180]) hinge_door();
        translate([42.5,-106,-10.5]) rotate([0,0,180]) hinge_cover();
    }
    terminal_block_bracket();
    translate([-111.4,-73,base_h]) rotate([0,0,-90]) probe_holder();
    translate([16,-38.4,base_h-2]) rotate([0,-90,0]) hinge_adapter();
    translate([13,-10,base_h-24.5]) angle_sensor_bracket();
    translate([16,90,base_h+6.9]) left_wrap_post();
    translate([-50,90,base_h+6.9]) right_wrap_post();
}


$fn=180;
// base();
// base_half();
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