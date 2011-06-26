#include <iostream>
#include <map>
#include <vector>
#include <string>

void Array2Vec(unsigned char arr[3], std::vector<unsigned char> &out);

void Colors();
void Strings();
void SetValue();
void SetAndGetValue();
void NonExistentValue();
void ResetValue();
void StringConst();
void IterateOverWholeMap();

int main(int argc, char *argv[])
{
  //SetValue();
  SetAndGetValue();
  //NonExistentValue();
  //ResetValue();

  //Strings();
  //Colors();
  // IterateOverWholeMap();
  return 0;
}

void SetValue()
{
  //create a map
  std::map <std::string, int> MyMap;

  //create a mapping from "test" to 111
  MyMap["test"] = 111;
}

void SetAndGetValue()
{
  //create a map
  std::map <std::string, int> MyMap;

  //create a mapping from "testone" to 111
  MyMap["testone"] = 111;

  //create an iterator
  std::map<std::string,int>::iterator iter;

  //try to find "testone"
  iter = MyMap.find("testone");

  //we assume "testone" was found, so output the value that "testone" maps to
  std::cout << iter->second << std::endl;
}

void ResetValue()
{
  std::string Name = "testone";
  std::map <std::string, int> MyMap;
  MyMap[Name] = 111;

  std::map<std::string,int>::iterator iter;

  iter = MyMap.find(Name);

  double FirstValue = iter->second;
  std::cout << FirstValue << std::endl;

  MyMap[Name] = FirstValue + 1;

  double SecondValue = iter->second;
  std::cout << SecondValue << std::endl;

}

void NonExistentValue()
{
  std::map <std::string, int> MyMap;
  MyMap["testone"] = 111;

  std::map<std::string,int>::iterator iter;

  iter = MyMap.find("testone");

  if(iter == MyMap.end())
  {
	  std::cout << "Element not found!" << std::endl;
  }
  else
  {
	  std::cout << "Element found!" << std::endl;
	  std::cout << iter->second << std::endl;
  }
}


void Strings()
{
  std::map <std::string, int> MyMap;
  MyMap["testone"] = 111;
  MyMap["testtwo"] = 222;
  MyMap["testseventyeight"] = 78;

  //tests
  std::cout << "Tests:" << std::endl;
  std::cout << MyMap["testtwo"] << std::endl;
  std::cout << MyMap["testseventyeight"] << std::endl;

  std::cout << MyMap["not in map 1"] << std::endl;
  std::cout << MyMap["not in map 2"] << std::endl;
/*
  if(MyMap["not in map"] == NULL)
	  cout << "NULL" << endl;

  if(MyMap["testone"] == NULL)
	  cout << "SHOULD NOT SEE THIS!" << endl;
*/
  std::map<std::string,int>::iterator iter;

  iter = MyMap.find("testone");
  if(iter == MyMap.end())
	  std::cout << "NO ELEMENT" << std::endl;
  else
	  std::cout << iter->second << std::endl;

  iter = MyMap.find("hello");
  if(iter == MyMap.end())
	  std::cout << "NO ELEMENT" << std::endl;
  else
	  std::cout << iter->second << std::endl;

  //get all items in the map
  std::cout << "All items:" << std::endl;

  std::map<std::string, int>::iterator alliter = MyMap.begin();
  for( ; alliter != MyMap.end(); ++alliter )
  {
	  std::cout << alliter->first << " " << alliter->second << std:: endl;
  }


}

void Colors()
{
  unsigned char r[3] = {255, 0, 0};
  unsigned char g[3] = {0, 255, 0};
  unsigned char b[3] = {0, 0, 255};

  std::vector<unsigned char> red(3,0);
  red[0] = 255;

  std::vector<unsigned char> green(3,0);
  green[1] = 255;

  std::vector<unsigned char> blue(3,0);
  blue[2] = 255;

  std::map<std::vector<unsigned char>, int> ColorList;
  ColorList[red] = 1;
  ColorList[green] = 2;
  ColorList[blue] = 3;

  std::cout << ColorList[red] << std::endl
		  << ColorList[green] << std::endl
		  << ColorList[blue] << std::endl;

  std::vector<unsigned char> RedVec(3);
  std::vector<unsigned char> GreenVec(3);
  std::vector<unsigned char> BlueVec(3);


  Array2Vec(r, RedVec);
  Array2Vec(g, GreenVec);
  Array2Vec(b, BlueVec);

  std::cout << ColorList[RedVec] << std::endl
		  << ColorList[GreenVec] << std::endl
		  << ColorList[BlueVec] << std::endl;

}
void Array2Vec(unsigned char arr[3], std::vector<unsigned char> &out)
{
  for(int i = 0; i <= 2; i++)
  {
    out[i] = arr[i];
  }

}

void IterateOverWholeMap()
{
  std::map <std::string, int> myMap;
  myMap["testone"] = 111;
  myMap["testtwo"] = 222;

  std::map<std::string,int>::iterator iter = myMap.begin();

  for(; iter != myMap.end(); ++iter)
  {
   std::cout << iter->first << " " << iter->second << std::endl;
  }
}