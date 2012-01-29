#include <QApplication>

#include <QMessage>
#include <QMessageAddress>
#include <QMessageService>
// http://doc.qt.nokia.com/qtmobility-1.2/qmessage.html
//#include "form.h"

//  By default, it would use the systems default mail account. There is a example "writemessage" that you might want to fiddle with. http://doc.qt.nokia.com/qtmobility-1.0-beta/writemessage.html

int main( int argc, char **argv )
{
  QApplication app(argc, argv);
//   MainWindow window;
//   
//   window.show();

  QMessageAddress fromAddress(QMessageAddress::Email, "daviddoria@gmail.com");
  QMessageAddress toAddress(QMessageAddress::Email, "hayleydoria@gmail.com");
  QMessage message;
  
  message->setFrom(fromAddress);
  message->setTo(toAddress);
  message->setBody("Test message.");
  message->setSubject("Test subject.");

  //message.setTo(QMessageAddress(QMessageAddress::Email, attendees[i]->email()));
  message.setType(QMessage::Email);
  //message.setDate(QDateTime::currentDateTime());
  //message.setFrom(QMessageAddress(QMessageAddress::Email, account->email()));
  //message.setSubject("Meeting invitation for " + attendees.at(i)->email());
  
  message.setParentAccountId(messageAccount.id());
  message.appendAttachments(QStringList()<<"invite.ics");
  bool sent = messageService.send(message);

  return app.exec();
}
