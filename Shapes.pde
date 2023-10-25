class Shape{
    public float mass, rot_inertia, angular_momentum, angle, total_torque;
    public Vec2 momentum, center, total_force;

    Shape (Vec2 center, float angle){
        this.mass = 1;
        this.rot_inertia = mass/12;
        this.angular_momentum = 0;
        this.angle = angle;
        this.total_torque = 0;
        this.momentum = new Vec2(0,0);
        this.center = center;
        this.total_force = new Vec2(0,0);
    }
}




class Line {
    Vec2 p1, p2;
    Line(Vec2 p1, Vec2 p2) {
        this.p1 = p1;
        this.p2 = p2;
    }
    boolean onLine(Line l1, Vec2 p) {
        if (p.x <= Math.max(l1.p1.x, l1.p2.x) && p.x >= Math.min(l1.p1.x, l1.p2.x) && p.y <= Math.max(l1.p1.y, l1.p2.y) && p.y >= Math.min(l1.p1.y, l1.p2.y)) return true;
        return false;
    }

    int direction(Vec2 a, Vec2 b, Vec2 c){
        float val = (b.y - a.y) * (c.x - b.x) - (b.x - a.x) * (c.y - b.y);
        
        if (val == 0) return 0;     // Collinear
    
        else if (val < 0) return 2; // Anti-clockwise direction
        
        return 1;                   // Clockwise direction
    }

    public int isIntersect(Line l1, Line l2) {
        // Four direction for two lines and points of other line
        int dir1 = direction(l1.p1, l1.p2, l2.p1);
        int dir2 = direction(l1.p1, l1.p2, l2.p2);
        int dir3 = direction(l2.p1, l2.p2, l1.p1);
        int dir4 = direction(l2.p1, l2.p2, l1.p2);
    
        // When intersecting
        if (dir1 != dir2 && dir3 != dir4)
        return 1;
    
        // When p2 of line2 are on the line1
        if (dir1 == 0 && onLine(l1, l2.p1))
        return 1;
    
        // When p1 of line2 are on the line1
        if (dir2 == 0 && onLine(l1, l2.p2))
        return 1;
    
        // When p2 of line1 are on the line2
        if (dir3 == 0 && onLine(l2, l1.p1))
        return 1;
    
        // When p1 of line1 are on the line2
        if (dir4 == 0 && onLine(l2, l1.p2))
        return 1;
    
        return 0;
    }
}

class Square extends Shape{
    Vec2 p1, p2, p3, p4;
    float w, h;
    Square(Vec2 center, float w, float h){
        super(center, 0.0);
        this.rot_inertia = mass*(w*w+h*h)/12;
        this.w = w;
        this.h = h;
    }

    Boolean isInside(Vec2 point){
        Vec2 pt = new Vec2(99999, point.y); // Create a point far off to the side and then check how many lines are intersected by the movement
        Line exit_line = new Line(point, pt);
        int count = 0;
        Line side = new Line(p1, p2);
        if (exit_line.isIntersect(exit_line, side) == 1) {
            if (exit_line.direction(p1, point, p2) == 0) return exit_line.onLine(side, point);
            count++;
        }
        side = new Line(p1, p3);
        if (exit_line.isIntersect(exit_line, side) == 1) {
            if (exit_line.direction(p1, point, p3) == 0) return exit_line.onLine(side, point);
            count++;
        }
        side = new Line(p2, p4);
        if (exit_line.isIntersect(exit_line, side) == 1) {
            if (exit_line.direction(p2, point, p4) == 0) return exit_line.onLine(side, point);
            count++;
        }
        side = new Line(p3, p4);
        if (exit_line.isIntersect(exit_line, side) == 1) {
            if (exit_line.direction(p3, point, p4) == 0) return exit_line.onLine(side, point);
            count++;
        }

        if (count == 1) return true;
        return false;
    }
}