//#include <QFont>
//#include <QtGui>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QFileDialog>

#include <iostream>

#include "form.h"

#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageRegionIterator.h"
#include "itkVectorImage.h"

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  setupUi(this);
}

void Form::on_btnOpen_clicked()
{
  QString fileName = QFileDialog::getOpenFileName(this, "Open File", ".", "Image Files (*.jpg *.jpeg *.bmp *.png)");

  std::cout << "Got filename: " << fileName.toStdString() << std::endl;
  if(fileName.toStdString().empty())
    {
    std::cout << "Filename was empty." << std::endl;
    return;
    }

  /*
  typedef itk::RGBPixel<unsigned char> RGBPixelType;
  typedef itk::Image<RGBPixelType> RGBImageType;
  
  typedef itk::ImageFileReader<RGBImageType> ReaderType;
  ReaderType::Pointer reader = ReaderType::New();
  reader->SetFileName(fileName.toStdString());
  reader->Update();
  
  RGBImageType::Pointer itkimage = reader->GetOutput();
  */
  
  //typedef itk::VectorImage<unsigned char, 2> ImageType;
  typedef itk::VectorImage<char, 2> ImageType;
  
  typedef itk::ImageFileReader<ImageType> ReaderType;
  ReaderType::Pointer reader = ReaderType::New();
  reader->SetFileName(fileName.toStdString());
  reader->Update();
  
  ImageType::Pointer itkimage = reader->GetOutput();
  
  QImage image(itkimage->GetLargestPossibleRegion().GetSize()[0], itkimage->GetLargestPossibleRegion().GetSize()[1], QImage::Format_RGB32);
  itk::ImageRegionIterator<ImageType> imageIterator(itkimage, itkimage->GetLargestPossibleRegion());
  QColor black(0,0,0);
  while(!imageIterator.IsAtEnd())
    {
    ImageType::PixelType pixel = imageIterator.Get();
    
    //QRgb qtPixel(pixel[0], pixel[1], pixel[2]);
    //image.setPixel(imageIterator.GetIndex()[0], imageIterator.GetIndex()[1], QColor(pixel[0], pixel[1], pixel[2]).rgb());
    image.setPixel(imageIterator.GetIndex()[0], imageIterator.GetIndex()[1], black.rgb());
    ++imageIterator;
    }

  
  
  //QPixmap image;
  //image.loadFromData(itkimage->GetBufferPointer());
  
  QGraphicsScene* scene = new QGraphicsScene();
  
  //scene->addPixmap(image);
  scene->addPixmap(QPixmap::fromImage(image));
  
  this->graphicsView->setScene(scene);
  
}

// Complicated text:
//QFont font("MS Serif", -1, -1, true);
//QFontInfo info(font);
//scene->addText("text", font);
  