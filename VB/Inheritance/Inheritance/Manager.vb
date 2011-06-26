Public Class Manager
    Inherits Employee

    Public Overrides Sub PrintMe()
        MessageBox.Show("manager: " + Name)
    End Sub
End Class
