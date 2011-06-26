Public Class GenericEmployee
    Private m_Name As String

    Public Property Name() As String
        Get
            Return m_Name
        End Get
        Set(ByVal value As String)
            m_Name = value
        End Set
    End Property

    Public Overridable Sub PrintMe()
        MessageBox.Show("Normal employee: " + Name)
    End Sub

End Class