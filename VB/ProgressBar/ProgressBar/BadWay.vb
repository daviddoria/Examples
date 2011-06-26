Public Class BadWay

    Dim WithEvents backgroundWorker1 As New System.ComponentModel.BackgroundWorker

    Private Sub backgroundWorker1_DoWork(ByVal sender As Object, ByVal e As System.ComponentModel.DoWorkEventArgs) Handles backgroundWorker1.DoWork

        MessageBox.Show(txtText.Text)

        'just do some computations to take some time
        For i As Integer = 1 To 10000000.0
            Dim j As Double = Math.Sin(i)
        Next

    End Sub

    Private Sub backgroundWorker1_RunWorkerCompleted(ByVal sender As Object, ByVal e As System.ComponentModel.RunWorkerCompletedEventArgs) Handles backgroundWorker1.RunWorkerCompleted
        frmWorking.Dispose()

        Button1.Text = "Run"
        Button1.Enabled = True

    End Sub

    Private Sub Button1_Click(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles Button1.Click
        Button1.Text = "Generating..."
        Button1.Enabled = False

        frmWorking.Show(Me)

        backgroundWorker1.RunWorkerAsync()
    End Sub

End Class
