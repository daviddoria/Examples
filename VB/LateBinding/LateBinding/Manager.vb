Public Class Manager
    Inherits Employee

    Public Overrides Sub PrintMe()
        MessageBox.Show("manager: " + Name)
    End Sub

    Public Sub ShowBoss()
        MessageBox.Show("Boss")
    End Sub

End Class
