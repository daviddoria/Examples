#include "itkLineSpatialObject.h"
#include "itkLineSpatialObjectPoint.h"

#include <set>

#include <vgl/vgl_point_2d.h>
#include <vgl/vgl_point_3d.h>
#include <vgl/vgl_line_segment_2d.h>
#include <vgl/vgl_line_segment_3d.h>
#include <vgl/vgl_intersection.h>

typedef itk::LineSpatialObject< 2 >   LineType;

void CreateLine1(LineType::Pointer);
void CreateLine2(LineType::Pointer);

// Don't allow elements that are within 'tolerance' of each other to be added to the set
struct ToleranceComparison
{
  bool operator()(const itk::Index<2> s1, const itk::Index<2> s2) const
  {
    double tolerance = 1e-4;
    if((s1[0] < s2[0]) && !( fabs(s1[0] - s2[0]) < tolerance) &&
      (s1[1] < s2[1]) && !( fabs(s1[1] - s2[1]) < tolerance) )
    {
      return true;
    }
    else
    {
      return false;
    }
  }
};

typedef std::set<itk::Index<2>, ToleranceComparison > ToleranceSet;

ToleranceSet IntersectLines(LineType::Pointer, LineType::Pointer);

bool vgl_intersection(vgl_line_segment_2d<double> lineSegment0, vgl_line_segment_2d<double> lineSegment1, vgl_point_2d<double> &intersectionPoint)
{
  vgl_point_3d<double> lineSegment03DP0(lineSegment0.point1().x(), lineSegment0.point1().y(), 0);
  vgl_point_3d<double> lineSegment03DP1(lineSegment0.point2().x(), lineSegment0.point2().y(), 0);
  vgl_line_segment_3d<double> lineSegment03D(lineSegment03DP0, lineSegment03DP1);


  vgl_point_3d<double> lineSegment13DP0(lineSegment1.point1().x(), lineSegment1.point1().y(), 0);
  vgl_point_3d<double> lineSegment13DP1(lineSegment1.point2().x(), lineSegment1.point2().y(), 0);
  vgl_line_segment_3d<double> lineSegment13D(lineSegment13DP0, lineSegment13DP1);

  vgl_point_3d<double> intersectionPoint3D;
  bool isIntersect = vgl_intersection(lineSegment03D, lineSegment13D, intersectionPoint3D);

  intersectionPoint.set(intersectionPoint3D.x(), intersectionPoint3D.y());

  return isIntersect;
}


int main( int argc, char *argv[] )
{

  LineType::Pointer line1 = LineType::New();
  CreateLine1(line1);

  LineType::Pointer line2 = LineType::New();
  CreateLine2(line2);

  ToleranceSet intersections = IntersectLines(line1, line2);

  std::cout << "Intersections:" << std::endl;
  for(ToleranceSet::iterator it1 = intersections.begin(); it1 != intersections.end(); it1++)
  {
    std::cout << *it1 << std::endl;
  }
  /*
  for(unsigned int i = 0; i < intersections.size(); i++)
    {
    std::cout << intersections[i] << std::endl;
    }
  */

  return EXIT_SUCCESS;
}

void CreateLine1(LineType::Pointer line)
{
  // Create a list of points
  std::vector<LineType::LinePointType> points;
  for(unsigned int i = 0; i < 20; i++)
    {
    LineType::LinePointType point;
    point.SetPosition(10,i);
    points.push_back(point);
    }

  // Create a line from the list of points
  line->SetPoints(points);
}

void CreateLine2(LineType::Pointer line)
{
 // Create a list of points
  std::vector<LineType::LinePointType> points;
  for(unsigned int i = 0; i < 20; i++)
    {
    LineType::LinePointType point;
    point.SetPosition(i,1.5);
    points.push_back(point);
    }

  // Create a line from the list of points
  line->SetPoints(points);
}

ToleranceSet IntersectLines(LineType::Pointer line1, LineType::Pointer line2)
{
  ToleranceSet intersections;

  for(unsigned int line1segmentId = 0; line1segmentId < line1->GetNumberOfPoints() - 1; line1segmentId++)
    {
    for(unsigned int line2segmentId = 0; line2segmentId < line2->GetNumberOfPoints() - 1; line2segmentId++)
      {

      // Create a line segment
      LineType::SpatialObjectPointType* line1P0 = line1->GetPoint(line1segmentId);
      LineType::SpatialObjectPointType* line1P1 = line1->GetPoint(line1segmentId+1);

      vgl_point_2d<double> lineSegment0P0(line1P0->GetPosition()[0],line1P0->GetPosition()[1]);
      vgl_point_2d<double> lineSegment0P1(line1P1->GetPosition()[0],line1P1->GetPosition()[1]);
      vgl_line_segment_2d<double> lineSegment0(lineSegment0P0, lineSegment0P1);
      //std::cout << lineSegment0 << " ";

      // Create another line segment
      LineType::SpatialObjectPointType* line2P0 = line2->GetPoint(line2segmentId);
      LineType::SpatialObjectPointType* line2P1 = line2->GetPoint(line2segmentId+1);

      vgl_point_2d<double> lineSegment1P0(line2P0->GetPosition()[0],line2P0->GetPosition()[1]);
      vgl_point_2d<double> lineSegment1P1(line2P1->GetPosition()[0],line2P1->GetPosition()[1]);
      vgl_line_segment_2d<double> lineSegment1(lineSegment1P0, lineSegment1P1);
      //std::cout << lineSegment1 << std::endl;

      vgl_point_2d<double> intersectionPoint;
      bool isIntersect = vgl_intersection(lineSegment0, lineSegment1, intersectionPoint);

      if(isIntersect)
        {
        std::cout << "intersection of segments " << line1segmentId << " and " << line2segmentId << std::endl;
        std::cout << lineSegment0 << " " << lineSegment1 << std::endl;
        itk::Index<2> pixel;
        pixel[0] = round(intersectionPoint.x());
        pixel[1] = round(intersectionPoint.y());
        intersections.insert(pixel);
        }
      }
    }

  return intersections;
}