Public MustInherit Class Employee
    Private m_Name As String

    Public Property Name() As String
        Get
            Return m_Name
        End Get
        Set(ByVal value As String)
            m_Name = value
        End Set
    End Property

    Public MustOverride Sub PrintMe()

End Class
