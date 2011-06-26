Public Class Form1

    Private Sub btnParse_Click(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles btnParse.Click
        Dim strString As String = txtString.Text
        Dim strDelimiter As String = txtDelimiter.Text

        Dim Strings() As String = strString.Split(strDelimiter)

        For Each s As String In Strings
            txtResults.Text = txtResults.Text + s + Environment.NewLine
        Next

        'Access strings individually:
        MessageBox.Show(Strings(0))

    End Sub
End Class
