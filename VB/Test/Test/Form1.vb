Public Class Form1

    Private Sub Button1_Click(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles Button1.Click
        Dim m_Address As String = "192.168.0.100"
        Dim m_result As String = "FAIL"

        If My.Computer.Network.Ping(m_Address) Then
            m_Result = "PASS"
        Else
            m_Result = "FAIL"
        End If

    End Sub
End Class
