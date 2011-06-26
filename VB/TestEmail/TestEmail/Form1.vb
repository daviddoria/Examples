Public Class Form1


    Private Sub btnSendEmail_Click(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles btnSendEmail.Click
        Try
            Dim EmailFrom As String = "service@kwikkopy.com"
            Dim EmailTo As String = "daviddoria@gmail.com"
            Dim EmailSubject As String = "Kwik Kopy Network Status Report"

            'compile report
            Dim EmailBody As String = _
            "Ping 192.168.0.1 successful" + Environment.NewLine + _
            "Ping 192.168.0.2 failed" + Environment.NewLine + _
            "Ping google.com successful"

            Dim email As New Net.Mail.MailMessage(EmailFrom, EmailTo, EmailSubject, EmailBody)

            Dim emailClient As New Net.Mail.SmtpClient("smtp.gmail.com", 587) '465 or 587
            emailClient.EnableSsl = True
            emailClient.Credentials = New Net.NetworkCredential("daviddoria@gmail.com", "hayley101")

            emailClient.Send(email)


        Catch ex As Exception
            MessageBox.Show("uh oh, error!")
            End
        End Try

        MsgBox("Email sent.")
    End Sub

    Private Sub btnTestPOP_Click(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles btnTestPOP.Click
        'ConnectToPop("mail.comcast.net")
        'ConnectToPop("mail.comcast3.net")
        'ConnectToPop("www.google.com")
        'ConnectToPop("mail2.kwikkopy.com")

        PopTest("mail2.kwikkopy.com")
        'PopTest("mail.comcast3.net")
    End Sub

    Private Sub btnMAC_Click(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles btnMAC.Click

        For Each nic As System.Net.NetworkInformation.NetworkInterface In System.Net.NetworkInformation.NetworkInterface.GetAllNetworkInterfaces()
            MessageBox.Show(String.Format("The MAC address of {0} is{1}{2}", nic.Description, Environment.NewLine, nic.GetPhysicalAddress()))
        Next

    End Sub

    Private Sub btnReset_Click(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles btnReset.Click
        ResetTCP()


    End Sub

    Private Sub btnEmailKK_Click(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles btnEmailKK.Click
        Try
            Dim EmailFrom As String = "service@kwikkopy.com"
            Dim EmailTo As String = "daviddoria@gmail.com"
            Dim EmailSubject As String = "Kwik Kopy Test Email"

            'compile report
            Dim EmailBody As String = "Test email"

            Dim email As New Net.Mail.MailMessage(EmailFrom, EmailTo, EmailSubject, EmailBody)

            Dim emailClient As New Net.Mail.SmtpClient("mail2.kwikkopy.com", 25)
            emailClient.EnableSsl = True
            emailClient.Credentials = New Net.NetworkCredential("service@kwikkopy.com", "1497service")

            emailClient.Send(email)


        Catch ex As Exception
            MessageBox.Show("Error! " + ex.Message)
            End
        End Try

        MsgBox("Email sent.")
    End Sub

    Private Sub btnEmailComcast_Click(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles btnEmailComcast.Click
        Try
            Dim EmailFrom As String = "nickdoria@comcast.net"
            Dim EmailTo As String = "daviddoria@gmail.com"
            Dim EmailSubject As String = "Comcast Test Email"

            'compile report
            Dim EmailBody As String = "Test email"

            Dim email As New Net.Mail.MailMessage(EmailFrom, EmailTo, EmailSubject, EmailBody)

            Dim emailClient As New Net.Mail.SmtpClient("smtp.comcast.net", 25)
            emailClient.EnableSsl = True
            emailClient.Credentials = New Net.NetworkCredential("nickdoria", "573grandoaks")

            emailClient.Send(email)


        Catch ex As Exception
            MessageBox.Show("Error! " + ex.Message)
            End
        End Try

        MsgBox("Email sent.")
    End Sub
End Class
