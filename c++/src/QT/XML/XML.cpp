#include <QtCore>
#include <QtXml>

#include <iostream>
using namespace std;

int main(int argc, char *argv[])
{

  QCoreApplication app(argc, argv);
  QFile file("data.xml");
  if (!file.open(QIODevice::ReadOnly))
    {
    std::cout << "Could not open file!" << std::endl;
    return 1;
    }
  QByteArray content = file.readAll();
  QDomDocument doc;
  QString errorMessage;
  int line, col;
  if (!doc.setContent(content, &errorMessage, &line, &col))
    {
    cout << "Error in Line " << line << ", column " << col
    << ":" << qPrintable(errorMessage) << endl;
    return 1;
    }

  QDomDocumentType type = doc.doctype();
  cout << "Document type: " << qPrintable(type.name()) << endl;;
  QDomElement root = doc.documentElement();
  if (root.hasAttribute("xmlns")) 
    {
    QDomAttr attr = root.attributeNode("xmlns");
    cout << "xmlns: " << qPrintable(attr.value()) << endl;
    }
  if (root.hasAttribute("lang")) 
    {
    QDomAttr attr = root.attributeNode("lang");
    cout << "lang: " << qPrintable(attr.value()) << endl;
    }
  QDomNode node = root.firstChild();
  while(!node.isNull())
    {
    if(node.isElement()) 
      {
      QDomElement elem = node.toElement();
      cout << "Child of root node: " << qPrintable(elem.tagName()) <<
      endl;
      cout << "Its text: " << qPrintable(elem.text()) << endl;
      }
    node = node.nextSibling();
    }

  QDomNodeList anchors = doc.elementsByTagName("a");
  for(uint i = 0; i < anchors.length(); i++) 
    {
    QDomElement anchor = anchors.at(i).toElement();
    QString href = anchor.attribute("href");
    cout << qPrintable(href) << endl;
    }

  QDomElement examplecom = anchors.at(0).toElement();
  examplecom.parentNode().removeChild(examplecom);

  for(uint i = 0; i < anchors.length(); i++) 
    {
    QDomElement anchor = anchors.at(i).toElement();
    anchor.setTagName("b");
    anchor.removeAttribute("href");
    }


  QDomDocumentFragment fragment = doc.createDocumentFragment();
  QDomElement italic = doc.createElement("i");
  QDomText text = doc.createTextNode("some links for you:");
  italic.appendChild(text);
  QDomComment comment = doc.createComment("converted links:");
  fragment.appendChild(italic);
  fragment.appendChild(comment);
  QDomNode para = doc.elementsByTagName("p").at(0);
  para.insertBefore(fragment, para.firstChild());

  QFile outfile("out.xml");
  if (!outfile.open(QIODevice::WriteOnly)) 
    {
    cout << "Could not write file: "
    << qPrintable(outfile.fileName()) << endl;
    return 1;
    }
  QByteArray data = doc.toByteArray(2);
  outfile.write(data);
  outfile.close();
  // unicode string representation
  QString string = doc.toString(2);
  cout << qPrintable(string) << endl;
  
  return 0;
}
