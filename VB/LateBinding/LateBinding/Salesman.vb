Public Class Salesman

    Inherits Employee

    Public Overrides Sub PrintMe()
        MessageBox.Show("salesman: " + Name)
    End Sub

    Public Sub ShowSales()
        MessageBox.Show("Sales")
    End Sub

End Class
