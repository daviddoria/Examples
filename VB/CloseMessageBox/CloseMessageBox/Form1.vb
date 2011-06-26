Public Class Form1

    Private Sub Button1_Click(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles Button1.Click
        'If bMessageOpen = True Then
        '    frmMessage.Label1.Text = TextBox1.Text
        'Else
        '    frmMessage.Label1.Text = TextBox1.Text
        '    'frmMessage.ShowDialog()
        '    frmMessage.Show()
        '    bMessageOpen = True
        'End If

        frmMessage.Label1.Text = TextBox1.Text

        If bMessageOpen = False Then
            frmMessage.Show()
            bMessageOpen = True
        End If
    End Sub
End Class
