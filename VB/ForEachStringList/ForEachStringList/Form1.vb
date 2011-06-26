Public Class Form1

    Private Sub Button1_Click(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles Button1.Click
        Dim Strings As New List(Of String)
        Strings.Add("hello")
        Strings.Add("world")

        For Each Str As String In Strings
            MessageBox.Show(Str)

        Next
    End Sub
End Class
