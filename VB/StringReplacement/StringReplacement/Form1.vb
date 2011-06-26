Public Class Form1

    Private Sub btnReplace_Click(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles btnReplace.Click
        Dim strOriginal As String = txtOriginal.Text

        lblResult.Text = strOriginal.Replace("#1#", "replaced")
    End Sub
End Class
