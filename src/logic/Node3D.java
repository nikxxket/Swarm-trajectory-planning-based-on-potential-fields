package logic;

class Node3D {
    int x, y, z;
    Node3D parent;
    double g, h, f;

    Node3D(int x, int y, int z, Node3D p, double g, double h) {
        this.x = x; this.y = y; this.z = z;
        this.parent = p;
        this.g = g; this.h = h; this.f = g + h;
    }
}