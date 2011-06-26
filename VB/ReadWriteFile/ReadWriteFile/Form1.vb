Public Class Form1

    Private Sub btnWrite_Click(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles btnWrite.Click

        Dim sContents As String = TextBox1.Text

        Dim sErr As String = ""

        Dim bAns As Boolean = SaveTextToFile(sContents, "D:\Test.txt", sErr)
        If bAns Then
            Debug.WriteLine("File Saved!")
        Else
            Debug.WriteLine("Error Saving File: " & sErr)
        End If

    End Sub

    Private Sub btnRead_Click(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles btnRead.Click
        Dim sErr As String = ""
        Dim sContents As String
        sContents = GetFileContents("D:\test.txt", sErr)
        If sErr = "" Then
            Debug.WriteLine("Got file!")
        Else
            Debug.WriteLine("Error retrieving file: " & sErr)
        End If

        TextBox1.Text = sContents

        'this works, but it doesn't remove the new line chars
        'Dim str As String() = TextBox1.Text.Split(New [Char]() {CChar(vbCrLf)})
        'For Each s As String In str
        'MsgBox(s)
        'Next

        'also works, but does not remove the new line chars
        'Dim readlines As String()
        'readlines = TextBox1.Text.Split(Environment.NewLine)
        'For i As Integer = 0 To readlines.GetUpperBound(0)
        ''MessageBox.Show(readlines(i) + "<br />")
        'MessageBox.Show(readlines(i))
        'Next

        'works properly, removes the newline chars
        Dim arrRows As String() = Split(sContents, vbNewLine)
        For i As Integer = 0 To arrRows.GetUpperBound(0)
            MessageBox.Show(Split(arrRows(i), " ")(0))
        Next

    End Sub

    Private Sub btnAppend_Click(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles btnAppend.Click
        AppendToFile(TextBox1.Text + vbCrLf, "c:\test.txt")

    End Sub
End Class
