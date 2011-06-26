Public Class Form1
    Inherits System.Windows.Forms.Form

#Region " Windows Form Designer generated code "

    Public Sub New()
        MyBase.New()

        'This call is required by the Windows Form Designer.
        InitializeComponent()

        'Add any initialization after the InitializeComponent() call

    End Sub

    'Form overrides dispose to clean up the component list.
    Protected Overloads Overrides Sub Dispose(ByVal disposing As Boolean)
        If disposing Then
            If Not (components Is Nothing) Then
                components.Dispose()
            End If
        End If
        MyBase.Dispose(disposing)
    End Sub

    'Required by the Windows Form Designer
    Private components As System.ComponentModel.IContainer

    'NOTE: The following procedure is required by the Windows Form Designer
    'It can be modified using the Windows Form Designer.  
    'Do not modify it using the code editor.
    Friend WithEvents Button1 As System.Windows.Forms.Button
    Friend WithEvents txtName As System.Windows.Forms.TextBox
    Friend WithEvents Button2 As System.Windows.Forms.Button
    <System.Diagnostics.DebuggerStepThrough()> Private Sub InitializeComponent()
        Me.Button1 = New System.Windows.Forms.Button
        Me.txtName = New System.Windows.Forms.TextBox
        Me.Button2 = New System.Windows.Forms.Button
        Me.SuspendLayout()
        '
        'Button1
        '
        Me.Button1.Location = New System.Drawing.Point(8, 8)
        Me.Button1.Name = "Button1"
        Me.Button1.TabIndex = 0
        Me.Button1.Text = "Button1"
        '
        'txtName
        '
        Me.txtName.Location = New System.Drawing.Point(40, 56)
        Me.txtName.Name = "txtName"
        Me.txtName.TabIndex = 1
        Me.txtName.Text = ""
        '
        'Button2
        '
        Me.Button2.Location = New System.Drawing.Point(168, 168)
        Me.Button2.Name = "Button2"
        Me.Button2.TabIndex = 2
        Me.Button2.Text = "Button2"
        '
        'Form1
        '
        Me.AutoScaleBaseSize = New System.Drawing.Size(5, 13)
        Me.ClientSize = New System.Drawing.Size(292, 269)
        Me.Controls.Add(Me.Button2)
        Me.Controls.Add(Me.txtName)
        Me.Controls.Add(Me.Button1)
        Me.Name = "Form1"
        Me.Text = "Form1"
        Me.ResumeLayout(False)

    End Sub

#End Region

    Private Sub Button1_Click(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles Button1.Click

        Dim DatabaseConnection As New OleDb.OleDbConnection

        DatabaseConnection.ConnectionString = "PROVIDER=Microsoft.Jet.OLEDB.4.0;Data Source = C:\timetracker.mdb"
        DatabaseConnection.Open()

        Dim MyDataSet As New DataSet
        Dim MyDataAdapter As OleDb.OleDbDataAdapter

        Dim Query As String

        Query = "SELECT * FROM associate"
        'SELECT * FROM Table_Name
        'SELECT tblContacts.FirstName, tblContacts.Surname FROM tblContacts


        MyDataAdapter = New OleDb.OleDbDataAdapter(Query, DatabaseConnection)
        MyDataAdapter.Fill(MyDataSet, "associates")

        'txtName.Text = MyDataSet.Tables("associates").Rows(0).Item(1)
        txtName.Text = MyDataSet.Tables("associates").Rows(0).Item(2)
        'item(1) returns name
        'item(2) returns social security number

        'OR refer to field name 
        'ds.Tables("AddressBook").Rows(0).Item("FirstName")

        'you can also set values
        'ds.Tables("AddressBook").Rows(2).Item(1) = "Jane"

        Dim MyCommandBuilder As New OleDb.OleDbCommandBuilder(MyDataAdapter)

        MyDataAdapter.Update(MyDataSet)


        Dim dsNewRow As DataRow

        dsNewRow = MyDataSet.Tables("AddressBook").NewRow()

        dsNewRow.Item("FirstName") = "FIRST NAME"

        MyDataSet.Tables("AddressBook").Rows.Add(dsNewRow)

        MyDataAdapter.Update(MyDataSet, "AddressBook")

        'to DELETE
        'MyDataSet.Tables("AddressBook").Rows(X).Delete()


        DatabaseConnection.Close()


    End Sub

    Private Sub Button2_Click(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles Button2.Click
        Dim a As System.DateTime
        MsgBox(a.Now.ToShortTimeString)
        MsgBox(a.Now.ToShortDateString)
    End Sub
End Class
