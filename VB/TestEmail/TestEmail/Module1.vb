Module Module1

    WithEvents KillTimer As New Timers.Timer(2000)
    Dim tClient As New Net.Sockets.TcpClient()
    Dim TimedOut As Boolean

    Private Sub KillTimer_Elapsed(ByVal sender As Object, ByVal e As System.Timers.ElapsedEventArgs) Handles KillTimer.Elapsed
        tClient.Close()
        KillTimer.Stop()
        TimedOut = True
    End Sub

    Public Sub ResetTCP()
        tClient.Close()
        tClient = New Net.Sockets.TcpClient()
        TimedOut = False
    End Sub

    Public Function PopTest(ByVal PopServer As String) As Boolean
        TimedOut = False
        KillTimer.Start()
        tClient.Connect(PopServer, 110)

        KillTimer.Stop()

        If TimedOut = True Then
            MessageBox.Show("FAIL")
        Else
            MessageBox.Show("PASS")
        End If

        ResetTCP()

    End Function

    'Public Function PopTest(ByVal PopServer As String) As Boolean
    '    KillTimer.Start()
    '    Try
    '        tClient.Connect(PopServer, 110)
    '        MessageBox.Show("no exception")
    '    Catch ex As Exception
    '        MessageBox.Show(ex.Message)
    '    End Try

    '    MessageBox.Show("Finished connecting")


    '    Dim ns As Net.Sockets.NetworkStream = tClient.GetStream()

    '    Dim sr As IO.StreamReader = New IO.StreamReader(ns)
    '    If tClient.Connected Then
    '        Dim connectLine As String = sr.ReadLine()
    '        If connectLine.StartsWith("+OK") Then
    '            MessageBox.Show("Connected")
    '        Else
    '            MessageBox.Show(String.Format("Error Connecting: {0}", connectLine))
    '        End If
    '        tClient.Close()
    '    Else
    '        MessageBox.Show("could not connect")
    '    End If

    '    ns.Dispose()
    '    sr.Dispose()
    'End Function



    'Public Function PopTest(ByVal PopServer As String) As Boolean
    '    Dim t As New Net.Sockets.TcpClient()

    '    Try
    '        t.Connect(PopServer, 110) '43 seconds without timeout decrease
    '    Catch ex As Exception
    '        MessageBox.Show("Unable to connect.")
    '        Exit Function
    '    End Try

    '    Dim ns As Net.Sockets.NetworkStream = t.GetStream()

    '    Dim sr As IO.StreamReader = New IO.StreamReader(ns)
    '    If t.Connected Then
    '        Dim connectLine As String = sr.ReadLine()
    '        If connectLine.StartsWith("+OK") Then
    '            MessageBox.Show("Connected")
    '        Else
    '            MessageBox.Show(String.Format("Error Connecting: {0}", connectLine))
    '        End If
    '        t.Close()
    '    Else
    '        MessageBox.Show("could not connect")
    '    End If

    '    ns.Dispose()
    '    sr.Dispose()

    'End Function


    Public Function ConnectToPop(ByVal POP3server As String) As Integer

        Try
            Dim Server As New Net.Sockets.TcpClient(POP3server, 110)
            Dim NetStrm As System.Net.Sockets.NetworkStream = Server.GetStream
            Dim RdStrm As New IO.StreamReader(Server.GetStream)
        Catch exc As Exception
            MessageBox.Show("Error")
            MsgBox(exc.Message)

            Exit Function
        End Try

        MessageBox.Show("Success")

        'Dim user As String
        'user = "test@mavensystems.biz"
        'Dim data As String = "USER " + user.Trim + vbCrLf
        'Dim szData() As Byte = System.Text.Encoding.ASCII.GetBytes(data.ToCharArray())
        'NetStrm.Write(szData, 0, szData.Length)
        'Dim POPResponse As String
        'POPResponse = RdStrm.ReadLine
        'If POPResponse.Substring(0, 4) = "-ERR" Then
        '    MsgBox("Invalid user Name")
        '    Return -1
        'End If
        'Dim password As String
        'password = "test1234"
        'data = "PASS " & password & vbCrLf
        'szData = System.Text.Encoding.ASCII.GetBytes(data.ToCharArray())
        'NetStrm.Write(szData, 0, szData.Length)
        'POPResponse = RdStrm.ReadLine
        'If POPResponse.Substring(0, 4) = "-ERR" Then
        '    MsgBox("Invalid Passwprd")
        '    Return (-1)
        'End If
        'data = "STAT" + vbCrLf
        'szData = System.Text.Encoding.ASCII.GetBytes(data.ToCharArray())
        'NetStrm.Write(szData, 0, szData.Length)
        'POPResponse = RdStrm.ReadLine
        'If POPResponse.Substring(0, 4) = "-ERR" Then
        '    MsgBox("could not log your in")
        '    Return -1
        'End If
        'data = "Stat" + vbCrLf
        'szData = System.Text.Encoding.ASCII.GetBytes(data.ToCharArray())
        'NetStrm.Write(szData, 0, szData.Length)
        'POPResponse = RdStrm.ReadLine
        'If POPResponse.Substring(0, 4) = "-ERR" Then
        '    MsgBox("could not log your in")
        '    Return -1
        'End If
        'Dim parts() As String
        'parts = POPResponse.Split(" ")
        'Dim messages, totsize As Integer

        ''messages = parts(3)
        'messages = CInt(parts(1))
        'Return messages



    End Function

End Module
