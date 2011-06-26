Public Class frmWorking

    Private Sub frmWorking_Load(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles MyBase.Load
        Me.StartPosition = FormStartPosition.CenterScreen
        Me.ControlBox = False

        ProgressBar1.Style = ProgressBarStyle.Blocks
        ProgressBar1.Style = ProgressBarStyle.Marquee
    End Sub
End Class