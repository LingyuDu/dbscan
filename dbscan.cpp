#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree.h>
#include <pcl/kdtree/kdtree_flann.h>
 
using namespace std;
 
float eps = 0.05;//the radius for searching neighbor points of octree
int min_pets = 5;
int min_cluster_pts = 50;
 
class point
{
public:
	float x;
	float y;
	float z;
	int visited = 0;
	int cluster = 0;
        int index = 0;
        vector<int> corepts;
        vector<int> neighbor_pts;
	point() {}
	point(float a, float b, float c)
	{
		x = a;
		y = b;
		z = c;
	}
};
vector<point> corecloud;
vector<point> allcloud;
float distance(point a, point b) {
	return sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y) + (a.z - b.z)*(a.z - b.z));
}
 
int main(int argc, char** argv)
{
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);//initialize the point cloud
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_color(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::io::loadPCDFile<pcl::PointXYZ>("/home/lingyu/Desktop/dbscan/cloud_plane.pcd", *cloud);//load the pcd file
        float resolution = 0.5f;//resolution of octree
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);//initialize the octree
	octree.setInputCloud(cloud);
	octree.addPointsFromInputCloud();
        pcl::PCDWriter writer;
 
	size_t len = cloud->points.size();
        cout<< "start to convert to rgba" << endl;
        for (size_t i = 0; i < len; i++)     //convert the type of points to pointxyzrgba and class point
	{
            pcl::PointXYZRGBA pt_color;
            pt_color.x = cloud->points[i].x;
            pt_color.y = cloud->points[i].y;
            pt_color.z = cloud->points[i].z;
            pt_color.r = 255;
            pt_color.g = 255;
            pt_color.b = 255;
            (*cloud_color).push_back(pt_color);
            point pt = point(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
            allcloud.push_back(pt);
	}
        writer.write<pcl::PointXYZRGBA> ("cloud_color.pcd", *cloud_color, false);
        cout<< "converted to rgba" << endl;

        // find the core poins and put them into the corecloud(vector<point>)
	for (size_t i = 0; i < len; i++)
	{
            vector<int> radiussearch;//store the index of neighbor points
            vector<float> radiusdistance;//store the square of distance of neighbor points
            octree.radiusSearch(cloud->points[i], eps, radiussearch, radiusdistance);//neighborhood research of octree
            if (radiussearch.size() > min_pets)
            {
                (*cloud_color).points[i].r = 255;
                (*cloud_color).points[i].g = 0;
                (*cloud_color).points[i].b = 0;
                allcloud[i].index = i;         //store the index of core points in the allcloud
                corecloud.push_back(allcloud[i]);
                allcloud[i].neighbor_pts = radiussearch; //store the index of neighbor points in the allcloud
            }
	}
        writer.write<pcl::PointXYZRGBA> ("cloud_corepts.pcd", *cloud_color, false);
        pcl::PointCloud<pcl::PointXYZ>::Ptr corecloud1(new pcl::PointCloud<pcl::PointXYZ>);
	corecloud1->points.resize(corecloud.size());
        cout << "extracted the core points" << endl;
        //copy the coordinate of points of corecloud to that of corecloud1
        for (int i = 0; i < corecloud.size(); i++)
        {
            corecloud1->points[i].x = corecloud[i].x;
            corecloud1->points[i].y = corecloud[i].y;
            corecloud1->points[i].z = corecloud[i].z;
	}


        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree1(resolution);
        octree1.setInputCloud(corecloud1);
        octree1.addPointsFromInputCloud();

        //find the density reachable points for each core point
        for (int i = 0; i<corecloud.size(); i++)
        {
            vector<int> pointIdxNKNSearch;
            vector<float> pointRadiusSquaredDistance;
            octree1.radiusSearch(corecloud1->points[i], eps, pointIdxNKNSearch, pointRadiusSquaredDistance);
            for (int j = 0; j < pointIdxNKNSearch.size(); j++)
            {
                corecloud[i].corepts.push_back(pointIdxNKNSearch[j]);
            }
	}
        cout<< "find the neighbor core points" << endl;


        //change the value of cluster for each core point according to if the neighbor core point is density reachable
        int outcluster = 0;
        vector<int> cluster_pts_num;
        for (int i = 0; i<corecloud.size(); i++)
        {

            int pts_num = 0;
            stack<point*> ps;
            if (corecloud[i].visited == 1) continue;
            corecloud[i].cluster = outcluster;
            ps.push(&corecloud[i]);
            point *v;
            //change the value of cluster for each point
            while (!ps.empty())
            {
                v = ps.top();
                v->visited = 1;
                allcloud[v->index].visited = 1;
                for(int pts_i = 0; pts_i < allcloud[v->index].neighbor_pts.size(); pts_i++)
                {
                    if(allcloud[allcloud[v->index].neighbor_pts[pts_i]].visited == 1)
                        continue;
                    else
                    {
                        allcloud[allcloud[v->index].neighbor_pts[pts_i]].cluster = outcluster;
                        allcloud[allcloud[v->index].neighbor_pts[pts_i]].visited = 1;
                        pts_num++;
                    }

                }
                ps.pop();
                for (int j = 0; j<v->corepts.size(); j++)
                {
                    if (corecloud[v->corepts[j]].visited == 1)
                        continue;
                    corecloud[v->corepts[j]].cluster = corecloud[i].cluster;
                    corecloud[v->corepts[j]].visited = 1;
                    allcloud[corecloud[v->corepts[j]].index].cluster = outcluster;
                    allcloud[corecloud[v->corepts[j]].index].visited = 1;
                    pts_num++;
                    for(int pts_i = 0; pts_i < allcloud[corecloud[v->corepts[j]].index].neighbor_pts.size(); pts_i++)
                    {
                        if(allcloud[allcloud[corecloud[v->corepts[j]].index].neighbor_pts[pts_i]].visited == 1)
                            continue;
                        else
                        {
                            allcloud[allcloud[corecloud[v->corepts[j]].index].neighbor_pts[pts_i]].cluster = outcluster;
                            allcloud[allcloud[corecloud[v->corepts[j]].index].neighbor_pts[pts_i]].visited = 1;
                            pts_num++;
                        }

                    }
                    ps.push(&corecloud[v->corepts[j]]);
                }
            }
            cout << "cluster: " << outcluster << " points number: " << pts_num << endl;
            cluster_pts_num.push_back(pts_num);
            outcluster++;
	}

        int cluster_num = 0;
        for (int i = 0; i < cluster_pts_num.size(); i++)
        {
            if(cluster_pts_num[i] > min_cluster_pts)
            {
                cluster_num++;
            }
        }

        cout << "number of clusters: " << cluster_num << endl;

        vector<vector<int>> color;
        int color_step = 255 / cluster_num * 3;
        int color_r = 0;
        int color_g = cluster_num / 3;
        int color_b = cluster_num / 3;

        //set color for different clusters
        for (int i = 0; i < cluster_pts_num.size(); i++)
        {
            if(cluster_pts_num[i] > min_cluster_pts)
            {
                vector<int> color_rgb;
                if (color_r < cluster_num / 3 && color_g > 0)
                {
                    color_r++;
                    color_g--;
                }
                else if (color_g < cluster_num / 3 && color_b > 0)
                {
                    color_g++;
                    color_b--;
                }
                else
                {
                    color_b++;
                    color_r--;
                }
                color_rgb.push_back(color_r * color_step);
                color_rgb.push_back(color_g * color_step);
                color_rgb.push_back(color_b * color_step);
                color.push_back(color_rgb);
            }
            else
            {
                vector<int> color_rgb;
                color_rgb.push_back(255);
                color_rgb.push_back(255);
                color_rgb.push_back(255);
                color.push_back(color_rgb);
            }
        }

        //color different clusters
        for (size_t i = 0; i < len; i++)
        {
            if(cluster_pts_num[allcloud[i].cluster] > min_cluster_pts)
            {
                cloud_color->points[i].r = color[allcloud[i].cluster][0];
                cloud_color->points[i].g = color[allcloud[i].cluster][1];
                cloud_color->points[i].b = color[allcloud[i].cluster][2];
            }
        }

        writer.write<pcl::PointXYZRGBA> ("cloud_clusters.pcd", *cloud_color, false);


}
