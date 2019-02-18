#include <map>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

struct lessPoint2f
{
    bool operator()(const Point2f& lhs, const Point2f& rhs) const
    {
        return (lhs.x == rhs.x) ? (lhs.y < rhs.y) : (lhs.x < rhs.x);
    }
};

Mat delaunay(const Mat1f& points, int imRows, int imCols)
/// Return the Delaunay triangulation, under the form of an adjacency matrix
/// points is a Nx2 mat containing the coordinates (x, y) of the points
{
    map<Point2f, int, lessPoint2f> mappts;
    Mat1b adj(points.rows, points.rows, uchar(0));
    /// Create subdiv and insert the points to it
    Subdiv2D subdiv(Rect(0, 0, imCols, imRows));
    for (int p = 0; p < points.rows; p++)
    {
        float xp = points(p, 0);
        float yp = points(p, 1);
        Point2f fp(xp, yp);

        // Don't add duplicates
        if (mappts.count(fp) == 0)
        {
            // Save point and index
            mappts[fp] = p;

            subdiv.insert(fp);
        }
    }

    /// Get the number of edges
    vector<Vec4f> edgeList;
    subdiv.getEdgeList(edgeList);
    int nE = edgeList.size();
    /// Check adjacency
    for (int i = 0; i < nE; i++)
    {
        Vec4f e = edgeList[i];
        cout<<edgeList[i]<<endl;
        cin.get();
        Point2f pt0(e[0], e[1]);
        Point2f pt1(e[2], e[3]);
        if (mappts.count(pt0) == 0 || mappts.count(pt1) == 0) {
            // Not a valid point
            continue;
        }

        int idx0 = mappts[pt0];
        int idx1 = mappts[pt1];

        // Symmetric matrix
        adj(idx0, idx1) = 1;
        adj(idx1, idx0) = 1;
    }
    return adj;
}


int main()
{
    Mat1f points(10, 2);
    randu(points, 0, 99);
    int rows = 100, cols = 100;
    Mat3b im(rows, cols, Vec3b(0,0,0));
    Mat1b adj = delaunay(points, rows, cols);
    for (int i = 0; i < points.rows; i++)
    {
        int xi = points.at<float>(i, 0);
        int yi = points.at<float>(i, 1);

        /// Draw the edges
        for (int j = i + 1; j < points.rows; j++)
        {
            if (adj(i, j))
            {
                int xj = points(j, 0);
                int yj = points(j, 1);
                line(im, Point(xi, yi), Point(xj, yj), Scalar(255, 0, 0), 1);
            }
        }
    }
    for (int i = 0; i < points.rows; i++)
    {
        int xi = points(i, 0);
        int yi = points(i, 1);

        /// Draw the nodes
        circle(im, Point(xi, yi), 1, Scalar(0, 0, 255), -1);
    }

    imshow("im", im);
    waitKey();
    return 0;
}
