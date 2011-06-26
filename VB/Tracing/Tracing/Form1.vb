Public Class Form1

    Dim bTrace As Boolean = True
    'Dim bTrace As Boolean = False

    Private Sub Button1_Click(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles Button1.Click
        Dim CallingButton As String = DirectCast(sender, Button).Name ' get the name of the button that was clicked to trigger this event

        Trace.WriteLineIf(bTrace, "Clicked " + CallingButton)
        Trace.Flush()

    End Sub

    Private Sub Form1_Load(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles MyBase.Load
        Trace.Listeners.Add(New TextWriterTraceListener("output.txt"))
    End Sub
End Class
