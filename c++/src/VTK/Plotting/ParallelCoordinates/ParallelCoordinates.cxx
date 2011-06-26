#include <vtkRenderer.h>
#include <vtkAxis.h>
#include <vtkStringArray.h>
#include <vtkRenderWindow.h>
#include <vtkSmartPointer.h>
#include <vtkChartParallelCoordinates.h>
#include <vtkPlot.h>
#include <vtkTable.h>
#include <vtkFloatArray.h>
#include <vtkContextView.h>
#include <vtkContextScene.h>
#include <vtkRenderWindowInteractor.h>

#include <sstream>


std::vector<std::string> split(const std::string &s, char delim);

std::vector<std::string> names;
std::vector<std::string> mpg;
std::vector<std::string> cyl;
std::vector<std::string> dsp;
std::vector<std::string> hp;
std::vector<std::string> lbs;
std::vector<std::string> acc;
std::vector<std::string> year;
std::vector<std::string> origin;

void Parse(std::string filename);

int main(int argc, char*argv[])
{
  Parse(argv[1]);

  // Set up a 2D scene, add an XY chart to it
  vtkSmartPointer<vtkContextView> view =
    vtkSmartPointer<vtkContextView>::New();
  view->GetRenderer()->SetBackground(1.0, 1.0, 1.0);
  view->GetRenderWindow()->SetSize(800, 600);
  vtkSmartPointer<vtkChartParallelCoordinates> chart =
    vtkSmartPointer<vtkChartParallelCoordinates>::New();
  view->GetScene()->AddItem(chart);

  // Create a table with some points in it...
  vtkSmartPointer<vtkTable> table =
    vtkSmartPointer<vtkTable>::New();

  vtkSmartPointer<vtkFloatArray> mpgArray =
    vtkSmartPointer<vtkFloatArray>::New();
  mpgArray->SetName("Milage (MPG)");
  table->AddColumn(mpgArray);

  vtkSmartPointer<vtkFloatArray> cylinderArray =
    vtkSmartPointer<vtkFloatArray>::New();
  cylinderArray->SetName("# of Cylinders");
  table->AddColumn(cylinderArray);

  vtkSmartPointer<vtkFloatArray> displacementArray =
    vtkSmartPointer<vtkFloatArray>::New();
  displacementArray->SetName("Displacement (sq. in.)");
  table->AddColumn(displacementArray);

  vtkSmartPointer<vtkFloatArray> horsepowerArray =
    vtkSmartPointer<vtkFloatArray>::New();
  horsepowerArray->SetName("Horsepower (hp)");
  table->AddColumn(horsepowerArray);

    vtkSmartPointer<vtkFloatArray> lbsArray =
    vtkSmartPointer<vtkFloatArray>::New();
  lbsArray->SetName("Weight (pounds)");
  table->AddColumn(lbsArray);

  vtkSmartPointer<vtkFloatArray> accArray =
    vtkSmartPointer<vtkFloatArray>::New();
  accArray->SetName("Accel. (0-60mph)");
  table->AddColumn(accArray);

  vtkSmartPointer<vtkFloatArray> yearArray =
    vtkSmartPointer<vtkFloatArray>::New();
  yearArray->SetName("Year");
  table->AddColumn(yearArray);

  // Test charting with a few more points...

  table->SetNumberOfRows(mpg.size());

  for (int i = 0; i < mpg.size(); ++i)
  {
    //std::cout << "data point: " << i << std::endl;

    //std::cout << mpg[i] << " " << cyl[i] << " " << dsp[i] << " " << hp[i] << " " << lbs[i] << " " << acc[i] << " " << year[i] << std::endl;

    std::stringstream ss;
    float temp;

    ss << mpg[i];

    ss >> temp;

    table->SetValue(i, 0, temp);
    ss.clear();
    ss.str("");

    ss << cyl[i];

    ss >> temp;

    table->SetValue(i, 1, temp);
    ss.clear();
    ss.str("");

    ss << dsp[i];

    ss >> temp;

    table->SetValue(i, 2, temp);
    ss.clear();
    ss.str("");

    ss << hp[i];

    ss >> temp;

    table->SetValue(i, 3, temp);
    ss.clear();
    ss.str("");

    ss << lbs[i];

    ss >> temp;

    table->SetValue(i, 4, temp);
    ss.clear();
    ss.str("");

    ss << acc[i];

    ss >> temp;

    table->SetValue(i, 5, temp);
    ss.clear();
    ss.str("");

    ss << year[i];

    ss >> temp;

    table->SetValue(i, 6, temp);
    ss.clear();
    ss.str("");

    }

  chart->GetPlot(0)->SetInput(table);

  view->Render();

  vtkSmartPointer<vtkStringArray> blankLabels =
    vtkSmartPointer<vtkStringArray>::New();
  blankLabels->SetNumberOfValues(chart->GetAxis(0)->GetNumberOfTicks());
  chart->GetAxis(0)->SetTickLabels(blankLabels);

  view->Render();

  //view->GetRenderWindow()->SetMultiSamples(0);

  view->GetInteractor()->Initialize();
  view->GetInteractor()->Start();

  return EXIT_SUCCESS;
}


std::vector<std::string> split(const std::string &s, char delim)
{
  std::vector<std::string> elems;
    std::stringstream ss(s);
    std::string item;
    while(std::getline(ss, item, delim))
    {
        elems.push_back(item);
    }
    return elems;
}

void Parse(std::string filename)
{

  std::ifstream fin(filename.c_str());

  if(fin == NULL)
    {
    std::cout << "Cannot open file." << std::endl;
    }

  std::vector<std::string> lines;
  std::string line;

  while(getline(fin, line))
  {
    lines.push_back(line);
  }

  for(unsigned int i = 0; i < lines.size(); i++)
  //for(unsigned int i = 0; i < 1; i++)
    {
    std::string currentLine = lines[i];

    std::vector<std::string> words = split(currentLine, '{');
    currentLine = words[1];
    words.clear();
    words = split(currentLine, '}');
    currentLine = words[0];

    words = split(currentLine, ',');


    std::vector<std::vector<std::string> > things;
    for(unsigned int j = 0; j < words.size(); j++)
      {
      std::vector<std::string> thing = split(words[j], ':');
      if(thing[1].compare("undefined") == 0)
        {
        thing[1] = "0";
        }
      things.push_back(thing);

      }

    if(things.size() != 9)
      {
      std::cerr << "Error: size is " << things.size() << std::endl;
      exit(-1);
      }


    //std::cout << things[1][1] << " " << things[2][1] << " " << things[3][1] << std::endl;;

    names.push_back(things[0][1]);
    mpg.push_back(things[1][1]);
    cyl.push_back(things[2][1]);
    dsp.push_back(things[3][1]);
    hp.push_back(things[4][1]);
    lbs.push_back(things[5][1]);
    acc.push_back(things[6][1]);
    year.push_back(things[7][1]);
    origin.push_back(things[8][1]);
    }

}